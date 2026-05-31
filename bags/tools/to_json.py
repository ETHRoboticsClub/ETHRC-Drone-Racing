#!/usr/bin/env python3
"""Convert a preprocessed trajectory pickle (from preprocess_bag.py) into
the trajectory JSON schema.

If the pickle contains an ``rc_command`` array (raw Betaflight rcCommand from
``/joystick_inputs``), it is converted to physical IsaacLab actions
([roll_rate, pitch_rate, yaw_rate, thrust]) via from_betaflight.py and stored
in ``actions``. Otherwise ``actions`` is left empty.

Usage:
    python3 to_json.py <input.pkl> <output.json> [--description TEXT]
        [--rc-rate R] [--super-rate S] [--rc-expo E] [--thrust-max-n N]
        [--no-invert-pitch] [--no-invert-yaw] [--invert-roll]
"""
from __future__ import annotations

import argparse
import json
import pickle
import sys
from pathlib import Path

# Import the validated Betaflight->IsaacLab conversion from the sibling module.
sys.path.insert(0, str(Path(__file__).resolve().parent))
from from_betaflight import THRUST_MAX_N, convert_row


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("input_pkl", type=Path)
    ap.add_argument("output_json", type=Path)
    ap.add_argument("--description", default="Recorded trajectory from rosbag (Vicon + IMU + joystick).")
    # Betaflight rate-curve config (must match the FC the bag was recorded on).
    ap.add_argument("--rc-rate", type=float, default=1.0, help="Betaflight RC_RATE (default 1.0).")
    ap.add_argument("--super-rate", type=float, default=0.7, help="Betaflight SUPER_RATE (default 0.7).")
    ap.add_argument("--rc-expo", type=float, default=0.0, help="Betaflight RC_EXPO (default 0.0).")
    ap.add_argument("--thrust-max-n", type=float, default=THRUST_MAX_N, help="Max total thrust [N] (must match the env).")
    # Frame convention: Betaflight is FRD, the sim env is FLU. The repo's
    # compute_actions() maps sim->FC by negating pitch & yaw, so FC->sim needs
    # the same negation. Default ON; pass --no-invert-* to disable.
    ap.add_argument("--no-invert-pitch", action="store_false", dest="invert_pitch",
                    help="Do NOT negate pitch rate (default: negate, for FRD->FLU).")
    ap.add_argument("--no-invert-yaw", action="store_false", dest="invert_yaw",
                    help="Do NOT negate yaw rate (default: negate, for FRD->FLU).")
    ap.add_argument("--invert-roll", action="store_true", help="Negate roll rate (default off; roll is shared FRD/FLU).")
    ap.set_defaults(invert_pitch=True, invert_yaw=True)
    args = ap.parse_args()

    with open(args.input_pkl, "rb") as f:
        d = pickle.load(f)

    dt = 1.0 / float(d["rate_hz"])
    t = d["t"]
    pos = d["position"]
    quat = d["orientation"]
    vel = d["linear_velocity"]
    omega = d["angular_velocity"]
    rc = d.get("rc_command")  # (N, 4) raw rcCommand, or None

    actions = []
    if rc is not None:
        actions = [
            convert_row(
                row,
                rc_rate=args.rc_rate,
                super_rate=args.super_rate,
                rc_expo=args.rc_expo,
                thrust_max_n=args.thrust_max_n,
                invert_roll=args.invert_roll,
                invert_pitch=args.invert_pitch,
                invert_yaw=args.invert_yaw,
            )
            for row in rc
        ]

    out = {
        "metadata": {
            "dt": dt,
            "action_format": "physical",
            "action_order": "roll_pitch_yaw_thrust",
            "quaternion_order": "xyzw",
            "frame": "world",
            "description": args.description,
            "source": "betaflight_rcCommand" if actions else "none",
            "betaflight_rate_config": {
                "rc_rate": args.rc_rate,
                "super_rate": args.super_rate,
                "rc_expo": args.rc_expo,
            },
            "thrust_max_n": args.thrust_max_n,
            "axis_inversions": {
                "roll": bool(args.invert_roll),
                "pitch": bool(args.invert_pitch),
                "yaw": bool(args.invert_yaw),
            },
        },
        "actions": actions,
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
    print(f"wrote {args.output_json}  ({len(t)} samples, {len(actions)} actions)")


if __name__ == "__main__":
    main()
