"""Convert Betaflight ``rcCommand`` stick values to IsaacLab physical actions.

Betaflight rcCommand layout (as published on ``/joystick_inputs`` by the
betaflight_blackbox_driver, raw/unscaled)::

    rcCommand[0]  Roll      ≈ −500 … +500  (0 centered)
    rcCommand[1]  Pitch     ≈ −500 … +500
    rcCommand[2]  Yaw       ≈ −500 … +500
    rcCommand[3]  Throttle  ≈ 1000 … 2000

IsaacLab hovering env physical action layout (this codebase's order)::

    action = [roll_rate, pitch_rate, yaw_rate, thrust]

with ``action_format = "physical"`` → rates in rad/s, thrust in N.

What this module does:
  1. Throttle: linear PWM (1000–2000) → ``[0, THRUST_MAX_N]`` N.
  2. Roll / pitch / yaw stick: apply the Betaflight rate curve (rcRate,
     superRate, rcExpo) to convert a ±500 stick value into a target body
     rate in deg/s, then to rad/s.
  3. Optional axis sign flips for Betaflight (FRD) ↔ IsaacLab sim (FLU)
     convention mismatches (see ``--invert-*`` flags / ``convert_row``).

FRAME NOTE (validated against src/rl/rl/hovering_experiment.py):
  The training/sim env works in FLU (x-fwd, y-left, z-up); Betaflight works
  in FRD (x-fwd, y-right, z-down). ``compute_actions`` maps sim→Betaflight by
  negating pitch and yaw (roll is shared). Going Betaflight→sim therefore
  requires the SAME negation of pitch and yaw. The bag pipeline applies this
  by default; the standalone CLI defaults match Betaflight's own docstring
  (no inversion) for backward compatibility — pass ``--invert-pitch
  --invert-yaw`` to land in the sim FLU frame.

Standalone usage::

    python bags/tools/from_betaflight.py \\
        --input    my_betaflight_log.json \\
        --output   recordings/my_run.json \\
        --dt       0.002              # seconds between rcCommand samples
        --rc-rate  1.0  --super-rate 0.7  --rc-expo 0.0 \\
        --invert-pitch --invert-yaw   # to match the sim FLU frame

Input JSON::

    { "rcCommand": [[roll, pitch, yaw, throttle], ...] }
"""

from __future__ import annotations

import argparse
import json
import math
from pathlib import Path


# ---------------------------------------------------------------------------
# IsaacLab drone constants (must match the training env / replay_actions.py).
# ---------------------------------------------------------------------------
THRUST_MAX_N: float = 34.192


# ---------------------------------------------------------------------------
# Betaflight rate curves — port of applyBetaflightRates() from
# src/main/flight/rc_curves.c. Defaults match Betaflight stock config.
# ---------------------------------------------------------------------------
def betaflight_stick_to_rate_rad_s(
    rc_cmd: float,
    *,
    rc_rate: float = 1.0,
    super_rate: float = 0.7,
    rc_expo: float = 0.0,
) -> float:
    """Convert a Betaflight ``rcCommand`` stick value (≈ ±500) → target rate [rad/s]."""
    stick = max(-1.0, min(1.0, rc_cmd / 500.0))

    # rcExpo: blends linear vs cubic response (odd-symmetric, sign preserved).
    stick = (1.0 - rc_expo) * stick + rc_expo * (stick ** 3)

    # Stock Betaflight scales rcRate when > 2 (rarely used at hobbyist defaults).
    eff_rc_rate = rc_rate + 14.54 * max(0.0, rc_rate - 2.0)

    # Base curve (deg/s).
    angle_rate_deg = 200.0 * eff_rc_rate * stick

    # SuperRate: hyperbolic stretch near the stick ends.
    if super_rate > 0.0:
        denom = max(0.01, min(1.0, 1.0 - abs(stick) * super_rate))
        angle_rate_deg /= denom

    return math.radians(angle_rate_deg)


def betaflight_throttle_to_thrust_N(
    rc_throttle: float,
    *,
    thrust_max_n: float = THRUST_MAX_N,
    pwm_min: int = 1000,
    pwm_max: int = 2000,
) -> float:
    """Linear PWM 1000–2000 → ``[0, thrust_max_n]`` N. Clamped to range."""
    norm = (rc_throttle - pwm_min) / (pwm_max - pwm_min)
    norm = max(0.0, min(1.0, norm))
    return norm * thrust_max_n


def convert_row(
    row,
    *,
    rc_rate: float = 1.0,
    super_rate: float = 0.7,
    rc_expo: float = 0.0,
    thrust_max_n: float = THRUST_MAX_N,
    pwm_min: int = 1000,
    pwm_max: int = 2000,
    invert_roll: bool = False,
    invert_pitch: bool = False,
    invert_yaw: bool = False,
) -> list[float]:
    """Convert one ``[roll, pitch, yaw, throttle]`` rcCommand row to a physical
    ``[roll_rate, pitch_rate, yaw_rate, thrust]`` action (rad/s, rad/s, rad/s, N)."""
    if len(row) < 4:
        raise ValueError(f"rcCommand row has < 4 fields: {row}")
    roll = betaflight_stick_to_rate_rad_s(row[0], rc_rate=rc_rate, super_rate=super_rate, rc_expo=rc_expo)
    pitch = betaflight_stick_to_rate_rad_s(row[1], rc_rate=rc_rate, super_rate=super_rate, rc_expo=rc_expo)
    yaw = betaflight_stick_to_rate_rad_s(row[2], rc_rate=rc_rate, super_rate=super_rate, rc_expo=rc_expo)
    thrust = betaflight_throttle_to_thrust_N(row[3], thrust_max_n=thrust_max_n, pwm_min=pwm_min, pwm_max=pwm_max)

    if invert_roll:
        roll = -roll
    if invert_pitch:
        pitch = -pitch
    if invert_yaw:
        yaw = -yaw

    return [roll, pitch, yaw, thrust]


# ---------------------------------------------------------------------------
# CLI
# ---------------------------------------------------------------------------
def _parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser(description="Convert a Betaflight rcCommand log to an IsaacLab replay JSON.")
    p.add_argument("--input", type=str, required=True, help="Input JSON with an 'rcCommand' list of [roll, pitch, yaw, throttle].")
    p.add_argument("--output", type=str, required=True, help="Output replay JSON.")
    p.add_argument("--dt", type=float, default=0.002, help="Sample period [s] (Betaflight blackbox is ~500 Hz by default → 0.002).")

    p.add_argument("--rc-rate", type=float, default=1.0, help="Betaflight RC_RATE (default 1.0).")
    p.add_argument("--super-rate", type=float, default=0.7, help="Betaflight SUPER_RATE (default 0.7).")
    p.add_argument("--rc-expo", type=float, default=0.0, help="Betaflight RC_EXPO (default 0.0).")

    p.add_argument("--thrust-max-n", type=float, default=THRUST_MAX_N, help="Max total thrust [N] for the simulated drone (must match the env).")
    p.add_argument("--pwm-min", type=int, default=1000, help="Throttle PWM min.")
    p.add_argument("--pwm-max", type=int, default=2000, help="Throttle PWM max.")

    # Sign flips for Betaflight (FRD) → IsaacLab sim (FLU) mismatches.
    p.add_argument("--invert-roll", action="store_true", help="Negate roll-rate output.")
    p.add_argument("--invert-pitch", action="store_true", help="Negate pitch-rate output (needed for FRD→FLU).")
    p.add_argument("--invert-yaw", action="store_true", help="Negate yaw-rate output (needed for FRD→FLU).")
    return p.parse_args()


def main() -> None:
    import numpy as np

    args = _parse_args()
    src = Path(args.input).resolve()
    data = json.loads(src.read_text())
    if "rcCommand" not in data:
        raise ValueError(f"{src} has no 'rcCommand' field (expected a list of [roll, pitch, yaw, throttle]).")
    rows = data["rcCommand"]

    actions: list[list[float]] = [
        convert_row(
            r,
            rc_rate=args.rc_rate,
            super_rate=args.super_rate,
            rc_expo=args.rc_expo,
            thrust_max_n=args.thrust_max_n,
            pwm_min=args.pwm_min,
            pwm_max=args.pwm_max,
            invert_roll=args.invert_roll,
            invert_pitch=args.invert_pitch,
            invert_yaw=args.invert_yaw,
        )
        for r in rows
    ]

    # Seed an identity-attitude initial state so a replay script has somewhere to
    # spawn (z = 2 m, zero velocities). The bag pipeline supplies a real trajectory.
    init_state = {
        "t": 0.0,
        "position": [0.0, 0.0, 2.0],
        "orientation": [0.0, 0.0, 0.0, 1.0],  # xyzw — identity
        "linear_velocity": [0.0, 0.0, 0.0],
        "angular_velocity": [0.0, 0.0, 0.0],
    }

    arr = np.asarray(actions, dtype=np.float32)
    peak_rates = np.abs(arr[:, :3]).max(axis=0)
    peak_thrust = arr[:, 3].max()
    print(f"[INFO] Converted {len(actions)} actions.")
    print(f"[INFO] Peak rates [rad/s]: roll={peak_rates[0]:.2f}, pitch={peak_rates[1]:.2f}, yaw={peak_rates[2]:.2f}")
    print(f"[INFO] Peak thrust [N]: {peak_thrust:.2f}  ({100*peak_thrust/args.thrust_max_n:.0f}% of max)")
    print("[INFO] NOTE: the env clamps actions to [-1, 1]. Rates beyond RATES_SCALE (default 9.0 rad/s)")
    print("       will saturate. Bump RATES_SCALE_OVERRIDE in replay_actions.py if needed.")

    payload = {
        "metadata": {
            "dt": float(args.dt),
            "action_format": "physical",
            "action_order": "roll_pitch_yaw_thrust",
            "quaternion_order": "xyzw",
            "frame": "world",
            "source": "betaflight_rcCommand",
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
        "trajectory": [init_state],
    }

    out = Path(args.output).resolve()
    out.parent.mkdir(parents=True, exist_ok=True)
    out.write_text(json.dumps(payload, indent=2))
    print(f"[INFO] Wrote {out}")


if __name__ == "__main__":
    main()
