#!/usr/bin/env python3
"""Convert a preprocessed trajectory pickle (from preprocess_bag.py) into
the trajectory JSON schema.

Usage:
    python3 to_json.py <input.pkl> <output.json> [--description TEXT]
"""
from __future__ import annotations

import argparse
import json
import pickle
from pathlib import Path


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("input_pkl", type=Path)
    ap.add_argument("output_json", type=Path)
    ap.add_argument("--description", default="Recorded trajectory from rosbag (Vicon + IMU).")
    args = ap.parse_args()

    with open(args.input_pkl, "rb") as f:
        d = pickle.load(f)

    dt = 1.0 / float(d["rate_hz"])
    t = d["t"]
    pos = d["position"]
    quat = d["orientation"]
    vel = d["linear_velocity"]
    omega = d["angular_velocity"]

    out = {
        "metadata": {
            "dt": dt,
            "action_format": "physical",
            "action_order": "roll_pitch_yaw_thrust",
            "quaternion_order": "xyzw",
            "frame": "world",
            "description": args.description,
        },
        "actions": [],
        "trajectory": [
            {
                "t": float(t[i]),
                "position": [float(x) for x in pos[i]],
                "orientation": [float(x) for x in quat[i]],
                "linear_velocity": [float(x) for x in vel[i]],
                "angular_velocity": [float(x) for x in omega[i]],
            }
            for i in range(len(t))
        ],
    }

    with open(args.output_json, "w") as f:
        json.dump(out, f, indent=2)
    print(f"wrote {args.output_json}  ({len(t)} samples)")


if __name__ == "__main__":
    main()
