"""ROS2 detector node — v3.4 hybrid gate-detection pipeline.

Per-frame flow:

1. Subscribe to ``/cam0/image_rgb`` (sensor_msgs/Image, BEST_EFFORT QoS so
   slow inference drops stale frames instead of building a backlog).
2. Subscribe once to ``/cam0/camera_info`` for real intrinsics +
   distortion model (equidistant / fisheye on zephyrus; plumb_bob on
   other rigs).
3. Run :class:`GatePredictor` (DeepLabV3+ seg + kp + ng heads) on the
   incoming BGR frame at 384×384.
4. Step :class:`HybridSegNextGateTracker` to get the **next gate's** four
   inner-corner pixel coordinates (TL, TR, BR, BL channel order).
5. Rescale corners from 384×384 to native camera resolution; undistort
   with ``cv2.fisheye.undistortPoints`` (or ``cv2.undistortPoints`` for
   plumb_bob); run ``cv2.solvePnP`` against the gate's 3D inner-corner
   model → ``cam_T_gate`` (rvec, tvec).
6. Compose the full transform chain
   ``world_T_body = world_T_gate · gate_T_cam · cam_T_body`` and publish
   a PoseWithCovarianceStamped on ``/gate_detection/drone_pose`` for the
   robot_localization EKF (see ``src/perception/ekf/config/rl_ekf.yaml``
   — that's the pose0 contract).

Stubs (CLAUDE.md §5 — both logged loudly on first use, NOT silent):

* ``world_T_gate`` defaults to identity. Without a parkour map we cannot
  know where the currently-locked gate sits in the world frame. The
  published pose IS structurally correct, but its world-frame origin is
  arbitrary until the map is wired in. Flying with this stub WILL crash
  the drone.
* ``cam_T_body`` defaults to the FPV convention: camera optical axis
  aligned with body +X (forward), camera right = body -Y, camera down =
  body -Z (REP-103). Translation assumed zero (camera at body origin).
  Replace with measured extrinsics before flight.

Every failure path emits a ``[WARN]`` via ``rclpy`` logging — no silent
frame drops.
"""

from __future__ import annotations

import os
from typing import Optional

import cv2
import numpy as np
import rclpy
import torch
from ament_index_python.packages import get_package_share_directory
from cv_bridge import CvBridge
from geometry_msgs.msg import PoseWithCovarianceStamped
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)
from scipy.spatial.transform import Rotation
from sensor_msgs.msg import CameraInfo, Image

from gate_detection.v3_4.inference.hybrid_tracker import HybridSegNextGateTracker
from gate_detection.v3_4.inference.predictor import GatePredictor


# --- Gate inner-rect 3D model ---------------------------------------------
# The v3.4 keypoint head emits 4 channels in CORNER_TYPES inner-half order:
# (inner_TL, inner_TR, inner_BR, inner_BL). The 3D model below matches that
# order so the 2D↔3D correspondence into cv2.solvePnP is correct.
#
# Gate frame convention: origin on the ground at the gate's centre-line,
# +X = gate-normal (drone fly-through direction), +Y = left, +Z = up.
# Inner-rectangle dimensions inherited from the legacy detector node
# (`half_w=0.85, h_top=1.8, h_bot=0.25` for the OUTER frame; the inner
# is inset by 10 cm on each side per the legacy code's approximation).
# TODO: confirm against the mechanical drawing for the actual competition
# gate; the values below match the legacy node so we don't regress.
_GATE_INNER_HALF_W: float = 0.75
_GATE_INNER_H_TOP: float = 1.7
_GATE_INNER_H_BOT: float = 0.35
_GATE_INNER_CORNERS_3D: np.ndarray = np.array([
    [0.0, +_GATE_INNER_HALF_W, _GATE_INNER_H_TOP],  # inner TL  (left-top)
    [0.0, -_GATE_INNER_HALF_W, _GATE_INNER_H_TOP],  # inner TR  (right-top)
    [0.0, -_GATE_INNER_HALF_W, _GATE_INNER_H_BOT],  # inner BR  (right-bottom)
    [0.0, +_GATE_INNER_HALF_W, _GATE_INNER_H_BOT],  # inner BL  (left-bottom)
], dtype=np.float32)


# --- cam→body extrinsic stub (FPV convention) ------------------------------
# Camera optical frame (REP-103): x=right, y=down, z=forward.
# Body frame (REP-103): x=forward, y=left, z=up.
# Mounting: camera nose-forward → cam optical z aligns with body +x.
# Column k of R_body_cam = the k-th basis vector of the camera frame
# expressed in body coords:
#   col 0 (cam +x = right) = body -y = (0, -1, 0)
#   col 1 (cam +y = down)  = body -z = (0, 0, -1)
#   col 2 (cam +z = fwd)   = body +x = (1, 0, 0)
_R_BODY_CAM_FPV: np.ndarray = np.array([
    [0.0, 0.0, 1.0],
    [-1.0, 0.0, 0.0],
    [0.0, -1.0, 0.0],
], dtype=np.float64)


def _se3(R: np.ndarray, t: np.ndarray) -> np.ndarray:
    """Pack a 3×3 rotation + (3,) translation into a 4×4 SE(3) matrix."""
    T = np.eye(4, dtype=np.float64)
    T[:3, :3] = R
    T[:3, 3] = t.reshape(3)
    return T


def _invert_se3(T: np.ndarray) -> np.ndarray:
    """Inverse of an SE(3) matrix (transpose of rotation, negate-rotate of trans)."""
    R = T[:3, :3]
    t = T[:3, 3]
    Tinv = np.eye(4, dtype=np.float64)
    Tinv[:3, :3] = R.T
    Tinv[:3, 3] = -R.T @ t
    return Tinv


class GateDetectorNode(Node):
    def __init__(self) -> None:
        super().__init__("gate_detector_node")

        # ---- parameters ---------------------------------------------------
        self.declare_parameter("checkpoint", "")
        self.declare_parameter("camera_topic", "/cam0/image_rgb")
        self.declare_parameter("camera_info_topic", "/cam0/camera_info")
        self.declare_parameter("output_topic", "/gate_detection/drone_pose")
        self.declare_parameter("device", "")
        self.declare_parameter("img_size", 384)
        self.declare_parameter("max_rate_hz", 30.0)
        # Covariance values published on the pose message. Conservative
        # diagonals; the EKF can be tuned from there once we have real
        # PnP-residual statistics from the zephyrus rig.
        self.declare_parameter("pose_position_std_m", 0.05)
        self.declare_parameter("pose_rotation_std_deg", 5.0)

        ckpt_path: str = self.get_parameter("checkpoint").get_parameter_value().string_value
        if not ckpt_path:
            ckpt_path = os.path.join(
                get_package_share_directory("gate_detection"),
                "checkpoints",
                "best_deeplabv3plus_keypoint_multi_balanced_v3_nextgate_seed42.pth",
            )
        if not os.path.isfile(ckpt_path):
            raise FileNotFoundError(
                f"v3.4 checkpoint not found at {ckpt_path!r}. Place the "
                f".pth file under the gate_detection package's "
                f"checkpoints/ directory, or pass -p checkpoint:=<path>."
            )

        device_param: str = self.get_parameter("device").get_parameter_value().string_value
        device: Optional[str] = device_param if device_param else None  # None → predictor auto-picks

        self.img_size: int = int(self.get_parameter("img_size").get_parameter_value().integer_value)

        self.get_logger().info(f"Loading v3.4 checkpoint: {ckpt_path}")
        self.predictor = GatePredictor(
            ckpt_path,
            img_size=self.img_size,
            device=device,
        )
        if not self.predictor.has_keypoint_head:
            raise RuntimeError(
                f"Checkpoint at {ckpt_path} has keypoint_head=False; the "
                f"hybrid tracker needs the 4-channel keypoint head + the "
                f"next-gate-mask head. Re-train with "
                f"model.keypoint_head=true and model.next_gate_head=true."
            )
        if not self.predictor.has_next_gate_head:
            self.get_logger().warning(
                f"[WARN] checkpoint at {ckpt_path}: next_gate_head=False. "
                f"Hybrid tracker will fall back to largest-CC on every "
                f"frame (logged separately by the tracker)."
            )
        if self.predictor.device == "cuda":
            torch.backends.cudnn.benchmark = True
        self.get_logger().info(
            f"Predictor ready (device={self.predictor.device}, "
            f"imagenet_norm={self.predictor.imagenet_norm}, "
            f"has_kp_head={self.predictor.has_keypoint_head}, "
            f"has_ng_head={self.predictor.has_next_gate_head})"
        )

        self.tracker = HybridSegNextGateTracker()

        self.bridge = CvBridge()

        # Rate-limit by min-period (preserved from prior detector_node).
        max_rate: float = float(self.get_parameter("max_rate_hz").get_parameter_value().double_value)
        self.min_period_ns: int = int(1e9 / max_rate) if max_rate > 0.0 else 0
        self.last_process_time = None

        # ---- camera intrinsics + distortion (latched on first message) ----
        self.K: Optional[np.ndarray] = None
        self.D: Optional[np.ndarray] = None
        self.distortion_model: Optional[str] = None
        self.cam_width: Optional[int] = None
        self.cam_height: Optional[int] = None
        # Latch camera_info: most camera drivers publish it with TRANSIENT_LOCAL
        # durability so late subscribers still see the latest value.
        info_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        info_topic = self.get_parameter("camera_info_topic").get_parameter_value().string_value
        self.info_sub = self.create_subscription(
            CameraInfo, info_topic, self._on_camera_info, info_qos,
        )

        # Image subscription with BEST_EFFORT + depth=1 so the inference
        # loop drops stale frames instead of building a backlog. Standard
        # ROS2 convention for sensor streams.
        sensor_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.BEST_EFFORT,
        )
        cam_topic = self.get_parameter("camera_topic").get_parameter_value().string_value
        self.image_sub = self.create_subscription(
            Image, cam_topic, self._on_image, sensor_qos,
        )

        output_topic = self.get_parameter("output_topic").get_parameter_value().string_value
        self.pose_pub = self.create_publisher(
            PoseWithCovarianceStamped, output_topic, 10,
        )

        # ---- WARN budgets so stub-fallback messages don't spam logs ------
        self._warned_stub_gate_world: bool = False
        self._warned_stub_cam_body: bool = False
        self._warned_no_camera_info: bool = False

        self.get_logger().info(
            f"GateDetectorNode up: image={cam_topic} info={info_topic} "
            f"output={output_topic} max_rate_hz={max_rate}"
        )

    # -----------------------------------------------------------------
    # Callbacks
    # -----------------------------------------------------------------
    def _on_camera_info(self, msg: CameraInfo) -> None:
        # Latch once; subsequent messages are ignored. (Destroying the
        # subscription from inside its own callback is documented as
        # fragile in rclpy, especially with MultiThreadedExecutor — the
        # flag-and-ignore pattern below is safer.)
        if self.K is not None:
            return
        # CameraInfo.K is a length-9 flat row-major 3x3.
        self.K = np.array(msg.k, dtype=np.float64).reshape(3, 3)
        self.D = np.array(msg.d, dtype=np.float64)
        self.distortion_model = msg.distortion_model
        self.cam_width = int(msg.width)
        self.cam_height = int(msg.height)
        self.get_logger().info(
            f"camera_info latched: {self.cam_width}x{self.cam_height} "
            f"distortion_model={self.distortion_model} "
            f"|D|={len(self.D)}"
        )

    def _on_image(self, msg: Image) -> None:
        # Rate-limit — same pattern as the prior detector_node.
        if self.min_period_ns > 0:
            now = self.get_clock().now()
            if self.last_process_time is not None:
                elapsed_ns = (now - self.last_process_time).nanoseconds
                if elapsed_ns < self.min_period_ns:
                    return
            self.last_process_time = now

        if self.K is None:
            if not self._warned_no_camera_info:
                self.get_logger().warning(
                    "[WARN] received image before camera_info; dropping "
                    "frame. (Further warnings suppressed.)"
                )
                self._warned_no_camera_info = True
            return

        # cv_bridge converts to BGR regardless of source encoding; the
        # predictor's _preprocess_bgr will RGB-convert internally.
        try:
            img_bgr = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except Exception as e:   # cv_bridge raises CvBridgeError on encoding issues
            self.get_logger().warning(
                f"[WARN] cv_bridge.imgmsg_to_cv2 failed "
                f"(encoding={msg.encoding!r}): {e}"
            )
            return

        H_cam, W_cam = img_bgr.shape[:2]

        # ---- forward pass + tracker --------------------------------------
        # fp16 autocast on CUDA gives ~1.4x speedup with no measurable
        # accuracy degradation on this model (Report 43: max abs Δseg
        # 5.2e-3, 100% kp argmax match, 99.996% binary mask agreement on
        # the TII test split). On CPU, autocast(enabled=False) is a
        # no-op so this stays correct on non-CUDA hosts.
        _use_amp = self.predictor.device.startswith("cuda")
        try:
            with torch.autocast(device_type="cuda", dtype=torch.float16, enabled=_use_amp):
                _rgb_resized, seg, kp, ng = self.predictor.predict_full_from_bgr(img_bgr)
        except Exception as e:
            self.get_logger().error(
                f"[ERROR] predictor.predict_full_from_bgr crashed: {e}"
            )
            return

        quad, status = self.tracker.step(seg, ng, kp)
        if quad is None:
            self.get_logger().info(
                f"no gate locked (reason={status.reason}, "
                f"candidates={status.n_candidates})"
            )
            return

        # ---- corner rescale 384 -> camera-native pixels ------------------
        # GatePredictor resizes the input to img_size×img_size, so the
        # detected quad lives in 384×384 coords. Rescale to native using a
        # half-pixel-aware mapping: a pixel center at coord x_in in an image
        # of width W_in corresponds to (x_in + 0.5) * W_out / W_in - 0.5 in
        # an image of width W_out. The naive `x_in * W_out / W_in` form is
        # off by up to ~1.7 px at 1280-wide native — invisible to overlays
        # but a systematic PnP-residual / cam_T_gate bias (Report 43 §7).
        sx = float(W_cam) / float(self.img_size)
        sy = float(H_cam) / float(self.img_size)
        quad_cam_px = quad.astype(np.float64).copy()
        quad_cam_px[:, 0] = (quad_cam_px[:, 0] + 0.5) * sx - 0.5
        quad_cam_px[:, 1] = (quad_cam_px[:, 1] + 0.5) * sy - 0.5

        # ---- undistort the corner pixels -> rectified image coords -------
        rectified = self._undistort_points(quad_cam_px)
        if rectified is None:
            return  # already logged

        # ---- PnP ---------------------------------------------------------
        # SOLVEPNP_IPPE — designed for general planar object points (the
        # gate inner rectangle is planar by construction, x_gate=0 for
        # all 4 corners). IPPE_SQUARE is NOT correct here: it requires a
        # unit-square arrangement with a specific axis convention; on a
        # 1.5×1.35 m rectangle it returns wrong poses (verified with a
        # synthetic roundtrip: terr=5.1 m vs ~0 with IPPE).
        try:
            success, rvec, tvec = cv2.solvePnP(
                _GATE_INNER_CORNERS_3D,
                rectified.reshape(-1, 1, 2).astype(np.float32),
                self.K,                # use original K — rectified pts are in original-K pixel coords
                np.zeros(5, dtype=np.float64),
                flags=cv2.SOLVEPNP_IPPE,
            )
        except cv2.error as e:
            self.get_logger().warning(f"[WARN] solvePnP raised: {e}")
            return

        if not success:
            self.get_logger().warning(
                f"[WARN] solvePnP returned success=False "
                f"(corners={quad_cam_px.tolist()})"
            )
            return

        # ---- compose world_T_body = world_T_gate · gate_T_cam · cam_T_body
        R_cam_gate, _ = cv2.Rodrigues(rvec)
        cam_T_gate = _se3(R_cam_gate, tvec.reshape(3))
        gate_T_cam = _invert_se3(cam_T_gate)

        world_T_gate = self._world_T_gate_stub()
        cam_T_body = self._cam_T_body_stub()
        world_T_body = world_T_gate @ gate_T_cam @ cam_T_body

        # ---- publish PoseWithCovarianceStamped ---------------------------
        pos = world_T_body[:3, 3]
        rot = world_T_body[:3, :3]
        # scipy returns quaternion as (x, y, z, w) — same order as ROS.
        qx, qy, qz, qw = Rotation.from_matrix(rot).as_quat()

        out = PoseWithCovarianceStamped()
        out.header.stamp = msg.header.stamp   # propagate image timestamp for EKF time-sync
        out.header.frame_id = "world"
        out.pose.pose.position.x = float(pos[0])
        out.pose.pose.position.y = float(pos[1])
        out.pose.pose.position.z = float(pos[2])
        out.pose.pose.orientation.x = float(qx)
        out.pose.pose.orientation.y = float(qy)
        out.pose.pose.orientation.z = float(qz)
        out.pose.pose.orientation.w = float(qw)

        pos_std = float(self.get_parameter("pose_position_std_m").get_parameter_value().double_value)
        rot_std_rad = float(self.get_parameter("pose_rotation_std_deg").get_parameter_value().double_value) * np.pi / 180.0
        cov = np.zeros((6, 6), dtype=np.float64)
        cov[0, 0] = cov[1, 1] = cov[2, 2] = pos_std * pos_std
        cov[3, 3] = cov[4, 4] = cov[5, 5] = rot_std_rad * rot_std_rad
        out.pose.covariance = cov.flatten().tolist()

        self.pose_pub.publish(out)

    # -----------------------------------------------------------------
    # Distortion handling
    # -----------------------------------------------------------------
    def _undistort_points(self, pts_px: np.ndarray) -> Optional[np.ndarray]:
        """Undistort ``(N, 2)`` pixel coords. Returns rectified pixel
        coords in the ORIGINAL ``K`` (so PnP can be called with the
        original ``K`` and zero distortion). Returns ``None`` and logs
        on unsupported distortion model.
        """
        assert self.K is not None and self.D is not None  # noqa: S101 — guarded in caller
        model = (self.distortion_model or "").lower()
        # cv2 expects shape (N, 1, 2) float32/64 for the points.
        pts_in = pts_px.reshape(-1, 1, 2).astype(np.float64)

        if model in ("equidistant", "fisheye"):
            # Fisheye D is 4 coefficients (k1..k4). Some drivers publish
            # 5 zeros (plumb_bob convention) on a fisheye lens — warn.
            D4 = np.asarray(self.D, dtype=np.float64).reshape(-1, 1)
            if D4.shape[0] < 4:
                self.get_logger().warning(
                    f"[WARN] fisheye distortion_model but D has "
                    f"{D4.shape[0]} coeffs (<4); cannot undistort."
                )
                return None
            D4 = D4[:4]
            return cv2.fisheye.undistortPoints(pts_in, self.K, D4, P=self.K).reshape(-1, 2)

        if model in ("plumb_bob", "rational_polynomial", ""):
            # Standard radial-tangential. Empty model string is also handled
            # this way per the existing convention in this repo's other
            # perception nodes — but log it so the assumption is auditable.
            if model == "":
                self.get_logger().warning(
                    "[WARN] camera_info has empty distortion_model; "
                    "assuming plumb_bob."
                )
            return cv2.undistortPoints(
                pts_in, self.K, np.asarray(self.D, dtype=np.float64), P=self.K,
            ).reshape(-1, 2)

        self.get_logger().warning(
            f"[WARN] unsupported distortion_model={model!r}; "
            f"supported: 'equidistant', 'plumb_bob'."
        )
        return None

    # -----------------------------------------------------------------
    # Frame stubs (CLAUDE.md §5: announce once, loudly)
    # -----------------------------------------------------------------
    def _world_T_gate_stub(self) -> np.ndarray:
        """Identity — placeholder until a parkour map exists."""
        if not self._warned_stub_gate_world:
            self.get_logger().warning(
                "[WARN] world_T_gate=identity STUB in use. Drone pose "
                "is published in a gate-anchored frame, NOT a true "
                "world frame. Wire up the parkour-map gate lookup "
                "before any autonomous flight."
            )
            self._warned_stub_gate_world = True
        return np.eye(4, dtype=np.float64)

    def _cam_T_body_stub(self) -> np.ndarray:
        """FPV convention — camera optical z = body +x, identity translation."""
        if not self._warned_stub_cam_body:
            self.get_logger().warning(
                "[WARN] cam_T_body STUB in use: FPV convention (optical "
                "z = body +x), zero translation. Replace with measured "
                "extrinsics for zephyrus before flight."
            )
            self._warned_stub_cam_body = True
        # cam_T_body = inverse of body_T_cam. body_T_cam has rotation
        # _R_BODY_CAM_FPV and zero translation, so the inverse is the
        # transpose with zero translation.
        return _se3(_R_BODY_CAM_FPV.T, np.zeros(3, dtype=np.float64))


def main(args=None):
    rclpy.init(args=args)
    node = GateDetectorNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
