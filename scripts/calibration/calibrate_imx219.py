#!/usr/bin/env -S uv run --quiet
# /// script
# requires-python = ">=3.10"
# dependencies = [
#   "opencv-python>=4.8",
#   "numpy>=1.24",
#   "rosbags>=0.10",
#   "rosbags-image>=0.9",
#   "matplotlib>=3.7",
# ]
# ///
"""
IMX219 wide-angle intrinsics calibration from a ROS 2 bag.

Pipeline
--------
1. Read /cam0/image_raw frames from a rosbag2 directory.
2. Detect 9x6 checkerboard inner corners with cv2.findChessboardCorners
   (+ cornerSubPix refinement).
3. Fit Kannala-Brandt fisheye intrinsics with cv2.fisheye.calibrate
   (== Kalibr's pinhole-equi model). For a ~160-deg FOV lens this
   models edge distortion correctly; the standard 5-coeff radtan model
   does not.
4. Optionally also fit a plumb_bob/radtan model for downstream consumers
   that require the TII reference schema (mtx + 5-coeff dist).
5. Write JSON (TII-compatible field names plus extras) and a
   Kalibr/OpenVINS-compatible YAML to the output directory.
6. Save a reprojection-error histogram and a sample annotated frame
   for visual sanity check.

Why fisheye over radtan
-----------------------
IMX219 wide-angle ~160-deg HFOV. The 5-coeff plumb_bob model
diverges past ~120 deg and produces large residuals at the image
periphery, exactly where racing gates appear during turns. The
Kannala-Brandt 4-coeff equidistant model is the right tool. The
existing OpenVINS configs in this repo already use it for fisheye
sensors (rs_t265, uzhfpv_indoor).

Usage
-----
    ./calibrate_imx219.py --bag /path/to/bags/imx219_calib_20260425_141500 \\
                          --square-size-mm 35.0
    ./calibrate_imx219.py --bag <dir> --square-size-mm 34.7 --frame-stride 5

Outputs (default to ./camera_calibration/):
    imx219_720p_60fps.json                  - intrinsics + metadata (both models)
    imx219_720p_60fps_kalibr.yaml           - Kalibr-style YAML for OpenVINS
    reprojection_error_hist.png             - per-frame RMS error
    sample_detection.png                    - annotated frame for sanity check
"""
from __future__ import annotations

import argparse
import json
import sys
from dataclasses import dataclass, field, asdict
from datetime import datetime, timezone
from pathlib import Path
from typing import Iterator

import cv2
import numpy as np


# ---------- bag iteration ----------------------------------------------------

def iter_image_messages(
    bag_dir: Path, topic: str = "/cam0/image_raw", stride: int = 1,
) -> Iterator[np.ndarray]:
    """Yield BGR frames decoded from the given rosbag2 directory.

    Decoding is the expensive call, so we honour ``stride`` *before*
    deserializing — large speedup on long bags.
    """
    from rosbags.highlevel import AnyReader
    from rosbags.image import message_to_cvimage
    from rosbags.typesys import Stores, get_typestore

    # rosbags >=0.10 doesn't bundle type defs; ROS 2 bags don't carry them
    # in metadata.yaml either, so we need to inject the humble typestore.
    typestore = get_typestore(Stores.ROS2_HUMBLE)
    with AnyReader([bag_dir], default_typestore=typestore) as reader:
        connections = [c for c in reader.connections if c.topic == topic]
        if not connections:
            sys.exit(
                f"[ERROR] topic {topic!r} not in bag. "
                f"Available: {[c.topic for c in reader.connections]}"
            )
        logged_first = False
        index = 0
        for conn, _t, raw in reader.messages(connections=connections):
            if index % stride != 0:
                index += 1
                continue
            index += 1
            msg = reader.deserialize(raw, conn.msgtype)
            if not logged_first:
                print(f"[INFO] bag topic {topic!r}: msgtype={conn.msgtype}, "
                      f"first encoding={getattr(msg, 'encoding', '?')}")
                logged_first = True
            yield message_to_cvimage(msg, "bgr8")


# ---------- calibration ------------------------------------------------------

@dataclass
class FisheyeResult:
    K: np.ndarray
    D: np.ndarray
    rms: float

@dataclass
class RadtanResult:
    K: np.ndarray
    D: np.ndarray
    rms: float

@dataclass
class CalibrationRun:
    image_size: tuple[int, int]
    n_images_used: int
    n_frames_considered: int
    pattern_inner: tuple[int, int]
    square_size_mm: float
    bag_path: str
    framerate_hz: float
    fisheye: FisheyeResult
    radtan: RadtanResult
    per_frame_rms_fisheye: list[float] = field(default_factory=list)


def build_object_points(pattern: tuple[int, int], square_mm: float) -> np.ndarray:
    """Object points for one chessboard view, on Z=0, in metres.

    ``pattern`` is ``(inner_cols, inner_rows)`` — the same tuple passed to
    ``cv2.findChessboardCorners``. The grid iteration order matches OpenCV's
    corner-return order: X (cols) varies fastest, Y (rows) slowest.
    """
    inner_cols, inner_rows = pattern
    objp = np.zeros((inner_rows * inner_cols, 3), np.float64)
    objp[:, :2] = np.mgrid[0:inner_cols, 0:inner_rows].T.reshape(-1, 2)
    objp *= square_mm * 1e-3
    return objp


def _detect_one(
    gray: np.ndarray, pattern: tuple[int, int],
) -> tuple[bool, np.ndarray | None]:
    """Try SB detector first (robust on fisheye), fall back to classic."""
    sb_flags = cv2.CALIB_CB_NORMALIZE_IMAGE | cv2.CALIB_CB_EXHAUSTIVE | cv2.CALIB_CB_ACCURACY
    found, corners = cv2.findChessboardCornersSB(gray, pattern, flags=sb_flags)
    if found:
        return True, corners
    classic_flags = cv2.CALIB_CB_ADAPTIVE_THRESH | cv2.CALIB_CB_NORMALIZE_IMAGE
    found, corners = cv2.findChessboardCorners(gray, pattern, flags=classic_flags)
    return bool(found), corners if found else None


def detect_corners(
    bag_dir: Path,
    pattern: tuple[int, int],
    square_mm: float,
    frame_stride: int,
    max_frames: int,
    annotated_out: Path | None = None,
    debug_dir: Path | None = None,
) -> tuple[list[np.ndarray], list[np.ndarray], tuple[int, int], int, int, tuple[int, int]]:
    """Walk the bag, detect chessboard corners, return objpts/imgpts lists.

    The pattern orientation (e.g. 9x6 vs 6x9) is determined automatically
    on the first detected frame and locked in for the rest of the bag.
    """
    pattern_used: tuple[int, int] | None = None
    objp_template: np.ndarray | None = None

    objpts: list[np.ndarray] = []
    imgpts: list[np.ndarray] = []
    img_size: tuple[int, int] | None = None

    considered = 0
    used = 0
    sample_saved = False
    debug_saved = 0
    subpix_criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 1e-3)

    for frame in iter_image_messages(bag_dir, stride=frame_stride):
        considered += 1
        if img_size is None:
            img_size = (frame.shape[1], frame.shape[0])
        # Save up to 3 raw frames for offline diagnosis when nothing detects.
        if debug_dir is not None and debug_saved < 3:
            cv2.imwrite(str(debug_dir / f"raw_{considered:04d}.png"), frame)
            debug_saved += 1

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        if pattern_used is None:
            # Try landscape and portrait orientations of the pattern.
            for candidate in (pattern, (pattern[1], pattern[0])):
                found, corners = _detect_one(gray, candidate)
                if found:
                    pattern_used = candidate
                    objp_template = build_object_points(candidate, square_mm)
                    print(f"[INFO] locked pattern orientation = {candidate} "
                          f"({'as configured' if candidate == pattern else 'transposed'})")
                    break
            else:
                if considered % 20 == 0:
                    print(f"[INFO] still searching for pattern... {considered} frames inspected, "
                          f"0 detections", flush=True)
                continue
        else:
            found, corners = _detect_one(gray, pattern_used)
            if not found:
                continue

        n_inner = pattern_used[0] * pattern_used[1]
        if corners.shape[0] != n_inner:
            print(f"[WARN] corner count mismatch: expected={n_inner}, "
                  f"got={corners.shape[0]}, fallback=skip-frame", flush=True)
            continue
        corners = cv2.cornerSubPix(gray, corners, (5, 5), (-1, -1), subpix_criteria)
        objpts.append(objp_template.copy())
        imgpts.append(corners)
        used += 1
        if used % 10 == 0:
            print(f"[INFO] {used} frames detected so far ({considered} inspected)", flush=True)

        if annotated_out is not None and not sample_saved and used >= 5:
            vis = frame.copy()
            cv2.drawChessboardCorners(vis, pattern_used, corners, True)
            cv2.imwrite(str(annotated_out), vis)
            sample_saved = True

        if used >= max_frames:
            break

    if img_size is None:
        sys.exit("[ERROR] no frames decoded from bag")
    if used < 10:
        hint = ""
        if debug_dir is not None and debug_saved > 0:
            hint = (f"\n[HINT] sample raw frames saved to {debug_dir}/raw_*.png. "
                    f"scp them to your laptop and inspect: is the board fully visible? "
                    f"Are squares clearly distinct? Bad lighting / motion blur / "
                    f"partial board / wrong pattern size are common causes.")
        sys.exit(f"[ERROR] only {used} frames had detectable corners. "
                 f"Need >=10. Re-record with better coverage.{hint}")

    assert pattern_used is not None
    return objpts, imgpts, img_size, considered, used, pattern_used


def fit_fisheye(
    objpts: list[np.ndarray], imgpts: list[np.ndarray], img_size: tuple[int, int],
) -> tuple[FisheyeResult, list[float]]:
    """Kannala-Brandt fit.

    cv2.fisheye.calibrate wants per-view shapes:
      objectPoints: (N, 1, 3) float64
      imagePoints:  (N, 1, 2) float64
    (or equivalently (1, N, 3) / (1, N, 2). The 4D shape (1, N, 1, 2)
    that an earlier version of this script used does NOT work — it
    fails the type assertion in fisheye.cpp.)
    """
    objpts_fe = [np.ascontiguousarray(op).reshape(-1, 1, 3).astype(np.float64) for op in objpts]
    imgpts_fe = [np.ascontiguousarray(ip).reshape(-1, 1, 2).astype(np.float64) for ip in imgpts]

    K = np.zeros((3, 3))
    D = np.zeros((4, 1))
    rvecs = [np.zeros((1, 1, 3), dtype=np.float64) for _ in objpts_fe]
    tvecs = [np.zeros((1, 1, 3), dtype=np.float64) for _ in objpts_fe]
    # CALIB_CHECK_COND deliberately omitted: it raises on a single
    # ill-conditioned frame and aborts the whole fit, which is
    # catastrophic with ~80 frames. Outliers are flagged via per-frame
    # RMS post-hoc instead.
    flags = cv2.fisheye.CALIB_RECOMPUTE_EXTRINSIC | cv2.fisheye.CALIB_FIX_SKEW
    criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 100, 1e-6)
    rms, K, D, rvecs_out, tvecs_out = cv2.fisheye.calibrate(
        objpts_fe, imgpts_fe, img_size, K, D, rvecs, tvecs, flags, criteria
    )

    per_frame: list[float] = []
    for op_fe, ip_fe, rv, tv in zip(objpts_fe, imgpts_fe, rvecs_out, tvecs_out):
        proj, _ = cv2.fisheye.projectPoints(op_fe, rv, tv, K, D)
        err = np.linalg.norm(proj.reshape(-1, 2) - ip_fe.reshape(-1, 2), axis=1)
        per_frame.append(float(np.sqrt(np.mean(err ** 2))))

    return FisheyeResult(K=K, D=D, rms=float(rms)), per_frame


def fit_radtan(
    objpts: list[np.ndarray], imgpts: list[np.ndarray], img_size: tuple[int, int],
) -> RadtanResult:
    """Standard plumb_bob/radtan fit (matches TII reference schema).

    cv2.calibrateCamera (the non-fisheye one) requires float32 (Point3f
    for objects, Point2f for images), unlike fisheye.calibrate which
    accepts float64. Cast explicitly here.
    """
    objpts_rt = [np.ascontiguousarray(op).reshape(-1, 1, 3).astype(np.float32) for op in objpts]
    imgpts_rt = [np.ascontiguousarray(ip).reshape(-1, 1, 2).astype(np.float32) for ip in imgpts]
    rms, K, D, _, _ = cv2.calibrateCamera(objpts_rt, imgpts_rt, img_size, None, None)
    return RadtanResult(K=K, D=D.flatten(), rms=float(rms))


# ---------- output -----------------------------------------------------------

def write_json(run: CalibrationRun, out_path: Path) -> None:
    K_fe = run.fisheye.K
    D_fe = run.fisheye.D.flatten()
    K_rt = run.radtan.K
    D_rt = run.radtan.D.flatten()
    payload = {
        "$schema_note": (
            "TII-compatible field names (mtx, dist) carry the radtan/plumb_bob fit. "
            "The 'fisheye' block carries the recommended Kannala-Brandt model and "
            "should be preferred for downstream perception (OpenVINS, ArUco) on "
            "the IMX219 wide-angle. See README in this directory."
        ),
        "camera": "imx219_wide_angle",
        "resolution": [run.image_size[0], run.image_size[1]],
        "framerate_hz": run.framerate_hz,
        # ---- TII-compatible block (radtan / plumb_bob) ----
        "mtx": K_rt.tolist(),
        "dist": [D_rt.tolist()],
        # ---- recommended fisheye block (Kannala-Brandt) ----
        "fisheye": {
            "model": "pinhole-equi",
            "intrinsics": {
                "fx": float(K_fe[0, 0]),
                "fy": float(K_fe[1, 1]),
                "cx": float(K_fe[0, 2]),
                "cy": float(K_fe[1, 2]),
            },
            "K": K_fe.tolist(),
            "distortion_model": "equidistant",
            "distortion_coeffs": {
                "k1": float(D_fe[0]),
                "k2": float(D_fe[1]),
                "k3": float(D_fe[2]),
                "k4": float(D_fe[3]),
            },
            "D": D_fe.tolist(),
        },
        # ---- provenance ----
        "calibration": {
            "tool": "opencv cv2.fisheye.calibrate + cv2.calibrateCamera",
            "opencv_version": cv2.__version__,
            "pattern": {
                "type": "checkerboard",
                "inner_corners_cols": run.pattern_inner[0],
                "inner_corners_rows": run.pattern_inner[1],
                "square_size_mm": run.square_size_mm,
            },
            "n_images_used": run.n_images_used,
            "n_frames_considered": run.n_frames_considered,
            "rms_reprojection_error_px": {
                "fisheye_pinhole_equi": run.fisheye.rms,
                "radtan_plumb_bob": run.radtan.rms,
            },
            "bag_path": run.bag_path,
            "calibrated_at_utc": datetime.now(timezone.utc).isoformat(timespec="seconds"),
        },
    }
    out_path.write_text(json.dumps(payload, indent=2))


def write_kalibr_yaml(run: CalibrationRun, out_path: Path, rostopic: str = "/cam0/image_raw") -> None:
    K = run.fisheye.K
    D = run.fisheye.D.flatten()
    text = (
        "# Kalibr-style camera chain for the IMX219 wide-angle.\n"
        "# Drop-in compatible with OpenVINS configs in src/perception/VIO/open_vins/config/.\n"
        "# Distortion model = equidistant (Kannala-Brandt 4 coefficients).\n"
        "cam0:\n"
        "  cam_overlaps: []\n"
        "  camera_model: pinhole\n"
        f"  distortion_coeffs: [{D[0]:.10g}, {D[1]:.10g}, {D[2]:.10g}, {D[3]:.10g}]\n"
        "  distortion_model: equidistant\n"
        f"  intrinsics: [{K[0,0]:.10g}, {K[1,1]:.10g}, {K[0,2]:.10g}, {K[1,2]:.10g}]\n"
        f"  resolution: [{run.image_size[0]}, {run.image_size[1]}]\n"
        f"  rostopic: {rostopic}\n"
    )
    out_path.write_text(text)


def write_diagnostics(run: CalibrationRun, out_dir: Path) -> None:
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    fig, ax = plt.subplots(figsize=(6, 3.5))
    ax.hist(run.per_frame_rms_fisheye, bins=20, color="#444", edgecolor="white")
    ax.axvline(run.fisheye.rms, color="#d35400", linestyle="--", label=f"overall RMS = {run.fisheye.rms:.3f} px")
    ax.set_xlabel("per-frame RMS reprojection error (px)")
    ax.set_ylabel("frame count")
    ax.set_title(f"IMX219 fisheye fit  ({run.n_images_used} frames, {run.image_size[0]}x{run.image_size[1]})")
    ax.legend()
    fig.tight_layout()
    fig.savefig(out_dir / "reprojection_error_hist.png", dpi=140)
    plt.close(fig)


# ---------- main -------------------------------------------------------------

def main() -> int:
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--bag", type=Path, required=True, help="rosbag2 directory")
    p.add_argument("--square-size-mm", type=float, required=True, help="MEASURED printed square size (mm)")
    p.add_argument("--inner-cols", type=int, default=9)
    p.add_argument("--inner-rows", type=int, default=6)
    p.add_argument("--frame-stride", type=int, default=10, help="use every Nth frame")
    p.add_argument("--max-frames", type=int, default=80, help="cap on detected frames used in the fit")
    p.add_argument("--framerate-hz", type=float, default=60.0)
    p.add_argument("--out-dir", type=Path, default=Path("camera_calibration"))
    p.add_argument("--out-name", type=str, default="imx219_720p_60fps")
    args = p.parse_args()

    args.out_dir.mkdir(parents=True, exist_ok=True)
    annotated = args.out_dir / "sample_detection.png"
    debug_dir = args.out_dir / "debug"
    debug_dir.mkdir(parents=True, exist_ok=True)

    pattern = (args.inner_cols, args.inner_rows)
    print(f"[INFO] reading bag: {args.bag}")
    objpts, imgpts, img_size, considered, used, pattern_used = detect_corners(
        args.bag, pattern, args.square_size_mm, args.frame_stride, args.max_frames,
        annotated_out=annotated, debug_dir=debug_dir,
    )
    print(f"[INFO] detected corners in {used}/{considered} sampled frames "
          f"(image size = {img_size[0]}x{img_size[1]}, pattern = {pattern_used[0]}x{pattern_used[1]})")

    print(f"[INFO] fitting Kannala-Brandt fisheye model (cv2.fisheye.calibrate)...")
    fisheye, per_frame = fit_fisheye(objpts, imgpts, img_size)
    print(f"[INFO]   fisheye RMS = {fisheye.rms:.4f} px")

    print(f"[INFO] fitting radtan/plumb_bob (cv2.calibrateCamera) for TII-schema compatibility...")
    radtan = fit_radtan(objpts, imgpts, img_size)
    print(f"[INFO]   radtan  RMS = {radtan.rms:.4f} px")

    if radtan.rms < fisheye.rms:
        print(f"[WARN] radtan rms ({radtan.rms:.3f}) < fisheye rms ({fisheye.rms:.3f}). "
              f"Unexpected for a wide-angle lens; check pattern coverage at image edges.", flush=True)

    run = CalibrationRun(
        image_size=img_size,
        n_images_used=used,
        n_frames_considered=considered,
        pattern_inner=pattern_used,
        square_size_mm=args.square_size_mm,
        bag_path=str(args.bag.resolve()),
        framerate_hz=args.framerate_hz,
        fisheye=fisheye,
        radtan=radtan,
        per_frame_rms_fisheye=per_frame,
    )

    # Embed the fisheye RMS in the filename so multiple runs are
    # immediately distinguishable (and sortable). e.g. rms066px = 0.66 px.
    rms_suffix = f"rms{int(round(fisheye.rms * 100)):03d}px"
    base = f"{args.out_name}_{rms_suffix}"
    json_out = args.out_dir / f"{base}.json"
    yaml_out = args.out_dir / f"{base}_kalibr.yaml"
    write_json(run, json_out)
    write_kalibr_yaml(run, yaml_out)
    write_diagnostics(run, args.out_dir)

    print(f"[OK] wrote {json_out}")
    print(f"[OK] wrote {yaml_out}")
    print(f"[OK] wrote {args.out_dir / 'reprojection_error_hist.png'}")
    print(f"[OK] wrote {annotated}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
