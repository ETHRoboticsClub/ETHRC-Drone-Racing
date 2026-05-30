#!/usr/bin/env python3
"""Filter a rosbag to /vicon/boreas/boreas + /imu0, then build a
synchronized trajectory pickle with finite-difference linear_velocity.

Usage:
    python3 preprocess_bag.py <input_bag_dir> [--out-bag DIR] [--out-pkl FILE] [--rate HZ]

The output pickle contains a dict of numpy arrays:
    {
      't':                (N,)    seconds, starting at 0
      'position':         (N, 3)
      'orientation':      (N, 4)  xyzw
      'linear_velocity':  (N, 3)
      'angular_velocity': (N, 3)
    }
"""
from __future__ import annotations

import argparse
import os
import pickle
import shutil
import subprocess
import sys
import tempfile
from pathlib import Path

import numpy as np

import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message

VICON_TOPIC = "/vicon/boreas/boreas"
IMU_TOPIC = "/imu0"


def run_filter(input_bag: Path, output_bag: Path) -> None:
    """Invoke `ros2 bag convert` with a generated yaml that keeps only the
    Vicon + IMU topics."""
    if output_bag.exists():
        shutil.rmtree(output_bag)

    with tempfile.NamedTemporaryFile("w", suffix=".yml", delete=False) as f:
        f.write(
            "output_bags:\n"
            f"  - uri: {output_bag}\n"
            "    storage_id: sqlite3\n"
            f"    topics: [{VICON_TOPIC}, {IMU_TOPIC}]\n"
        )
        yaml_path = f.name

    try:
        subprocess.run(
            ["ros2", "bag", "convert", "-i", str(input_bag), "-o", yaml_path],
            check=True,
        )
    finally:
        os.unlink(yaml_path)


def read_streams(bag_dir: Path):
    """Read Vicon and IMU streams from a bag directory."""
    storage = rosbag2_py.StorageOptions(uri=str(bag_dir), storage_id="sqlite3")
    converter = rosbag2_py.ConverterOptions("", "")
    reader = rosbag2_py.SequentialReader()
    reader.open(storage, converter)

    type_map = {t.name: t.type for t in reader.get_all_topics_and_types()}

    vicon = []  # (t, pos[3], quat[4])
    imu = []  # (t, omega[3])

    while reader.has_next():
        topic, raw, _t_ns = reader.read_next()
        if topic not in (VICON_TOPIC, IMU_TOPIC):
            continue
        msg_cls = get_message(type_map[topic])
        msg = deserialize_message(raw, msg_cls)
        if topic == VICON_TOPIC:
            t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
            p = msg.pose.position
            q = msg.pose.orientation
            vicon.append((t, [p.x, p.y, p.z], [q.x, q.y, q.z, q.w]))
        else:
            t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
            w = msg.angular_velocity
            imu.append((t, [w.x, w.y, w.z]))

    vicon.sort(key=lambda x: x[0])
    imu.sort(key=lambda x: x[0])

    v_t = np.array([v[0] for v in vicon])
    v_p = np.array([v[1] for v in vicon])
    v_q = np.array([v[2] for v in vicon])

    i_t = np.array([v[0] for v in imu])
    i_w = np.array([v[1] for v in imu])
    return v_t, v_p, v_q, i_t, i_w


def vicon_velocity(v_t: np.ndarray, v_p: np.ndarray) -> np.ndarray:
    """Mirror state_bridge_node.py: raw = (p_cur - p_prev)/dt, then
    smoothed = 0.5 * smoothed_prev + 0.5 * raw."""
    n = len(v_t)
    vel = np.zeros_like(v_p)
    for k in range(1, n):
        dt = v_t[k] - v_t[k - 1]
        if dt > 0:
            raw = (v_p[k] - v_p[k - 1]) / dt
            vel[k] = 0.5 * vel[k - 1] + 0.5 * raw
        else:
            vel[k] = vel[k - 1]
    return vel


def slerp_batch(t_src: np.ndarray, q_src: np.ndarray, t_tgt: np.ndarray) -> np.ndarray:
    """Per-segment slerp. Assumes q xyzw."""
    out = np.zeros((len(t_tgt), 4))
    idx = np.searchsorted(t_src, t_tgt) - 1
    idx = np.clip(idx, 0, len(t_src) - 2)
    for k, i in enumerate(idx):
        t0, t1 = t_src[i], t_src[i + 1]
        u = 0.0 if t1 == t0 else (t_tgt[k] - t0) / (t1 - t0)
        u = float(np.clip(u, 0.0, 1.0))
        q0, q1 = q_src[i].copy(), q_src[i + 1].copy()
        if np.dot(q0, q1) < 0:
            q1 = -q1
        dot = float(np.clip(np.dot(q0, q1), -1.0, 1.0))
        if dot > 0.9995:
            q = (1 - u) * q0 + u * q1
        else:
            omega = np.arccos(dot)
            s = np.sin(omega)
            q = (np.sin((1 - u) * omega) / s) * q0 + (np.sin(u * omega) / s) * q1
        out[k] = q / np.linalg.norm(q)
    return out


def lerp_batch(t_src: np.ndarray, y_src: np.ndarray, t_tgt: np.ndarray) -> np.ndarray:
    out = np.empty((len(t_tgt), y_src.shape[1]))
    for d in range(y_src.shape[1]):
        out[:, d] = np.interp(t_tgt, t_src, y_src[:, d])
    return out


def resample(v_t, v_p, v_q, v_v, i_t, i_w, rate_hz: float):
    dt = 1.0 / rate_hz
    t0 = max(v_t[0], i_t[0])
    t1 = min(v_t[-1], i_t[-1])
    n = int(np.floor((t1 - t0) / dt)) + 1
    t_grid = t0 + np.arange(n) * dt

    pos = lerp_batch(v_t, v_p, t_grid)
    vel = lerp_batch(v_t, v_v, t_grid)
    quat = slerp_batch(v_t, v_q, t_grid)
    omega = lerp_batch(i_t, i_w, t_grid)
    return t_grid - t_grid[0], pos, quat, vel, omega


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("input_bag", type=Path, help="path to source bag directory")
    ap.add_argument("--out-bag", type=Path, default=None,
                    help="filtered bag output dir (default: <input>_filtered)")
    ap.add_argument("--out-pkl", type=Path, default=None,
                    help="preprocessed pickle path (default: <input>_traj.pkl)")
    ap.add_argument("--rate", type=float, default=100.0, help="resample rate (Hz)")
    ap.add_argument("--skip-filter", action="store_true",
                    help="skip ros2 bag convert; treat input_bag as already filtered")
    args = ap.parse_args()

    input_bag = args.input_bag.resolve()
    if not input_bag.is_dir():
        sys.exit(f"input bag dir not found: {input_bag}")

    if args.skip_filter:
        filtered = input_bag
    else:
        filtered = (args.out_bag or input_bag.parent / f"{input_bag.name}_filtered").resolve()
        print(f"[1/2] filtering -> {filtered}", flush=True)
        run_filter(input_bag, filtered)

    print(f"[2/2] reading + computing linear_velocity", flush=True)
    v_t, v_p, v_q, i_t, i_w = read_streams(filtered)
    if len(v_t) < 2 or len(i_t) < 2:
        sys.exit(f"insufficient samples (vicon={len(v_t)}, imu={len(i_t)})")
    v_v = vicon_velocity(v_t, v_p)

    t, pos, quat, vel, omega = resample(v_t, v_p, v_q, v_v, i_t, i_w, args.rate)

    out_pkl = (args.out_pkl or input_bag.parent / f"{input_bag.name}_traj.pkl").resolve()
    with open(out_pkl, "wb") as f:
        pickle.dump({
            "t": t,
            "position": pos,
            "orientation": quat,
            "linear_velocity": vel,
            "angular_velocity": omega,
            "rate_hz": args.rate,
        }, f)
    print(f"wrote {out_pkl}  ({len(t)} samples @ {args.rate} Hz)")


if __name__ == "__main__":
    main()
