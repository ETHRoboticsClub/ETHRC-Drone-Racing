"""
Offline test for DroneRLController.controller_update.

Loads a policy from a user-supplied path and runs it through four hand-crafted
hovering scenarios: the goal is always at [0, 0, 1] (1 m above world origin),
and the drone sits 1 m away from the goal in each of the four cardinal
horizontal directions, level, motionless, identity attitude.

Prints the AttitudeRate command produced by `controller_update` for each case.

No ROS, no MAVSDK runtime, no hardware. Run from anywhere (this script wires
its own sys.path so neither `src/rl` nor `src/interfaces` needs to be on
PYTHONPATH):

    python src/rl/rl/test_controller_offline.py models/policy.pt
"""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

# --- sys.path bootstrap ---------------------------------------------------
# Repo layout:
#   ETHRC-Drone-Racing/src/rl/rl/test_controller_offline.py   (this file)
#   ETHRC-Drone-Racing/src/rl/rl/                             → package "rl"
#   ETHRC-Drone-Racing/src/interfaces/interfaces/             → package "interfaces"
# Add the two parents that contain these packages so `import rl` and
# `import interfaces` both resolve regardless of where the script is invoked.
_THIS_FILE = Path(__file__).resolve()
_SRC_DIR = _THIS_FILE.parents[2]            # .../src
for _candidate in (_SRC_DIR / "rl", _SRC_DIR / "interfaces"):
    if str(_candidate) not in sys.path:
        sys.path.insert(0, str(_candidate))
# -------------------------------------------------------------------------

import numpy as np  # noqa: E402

# Imported after the sys.path bootstrap. Patching `_POLICY_PATH` happens later,
# in main(), before DroneRLController is constructed.
from rl import drone_rl_controller  # noqa: E402
from rl.hovering_experiment import ANG_COMMAND_SCALE # noqa: E402

_PKG_DIR = _THIS_FILE.parent

# State vector layout (must match drone_state_sim publishers):
#   [0:3]  pos x,y,z           (world)
#   [3:6]  vel x,y,z           (world)
#   [6:10] quat x,y,z,w        (xyzw)
#   [10:13] omega x,y,z        (body, rad/s)
GOAL = np.array([0.0, 0.0, 1.0], dtype=np.float64)

# Small upward z-velocity (m/s, world frame) seeded into every case.
# Rationale: at exact rest (v=0), a hover policy reads the state as "stationary
# and needs full active support against gravity" and tends to spike thrust on
# the first tick. Feeding a modest upward velocity simulates the drone already
# being supported, so the resulting AttitudeRate reflects what the policy would
# command in mid-flight rather than at the instant of liftoff.
_HOVER_VZ = 0.5

# Expectation encoding (one entry per case):
#   roll/pitch/yaw_sign:
#       -1 → command should be clearly negative
#       +1 → command should be clearly positive
#        0 → command should be near zero (|value| < ZERO_TOL_DEG_S)
#   With identity attitude the body frame equals the world frame in FLU, and
#   compute_actions then flips Y and Z to FRD before publishing — so the signs
#   below are the *output* (FRD, deg/s) signs you should see in the printout.
#       east (drone +X of goal): nose-up pitch in FLU → negative pitch in FRD.
#       west (drone -X of goal): nose-down pitch in FLU → positive pitch in FRD.
#       north (drone +Y, goal -Y): right roll in FLU → positive roll in FRD.
#       south (drone -Y, goal +Y): left roll in FLU → negative roll in FRD.
#       on-target: no positional error → no rotational command.
#
# (label, position [x,y,z], expected_signs={'roll':..,'pitch':..,'yaw':..})
CASES = [
    ("east   (drone +X of goal)", np.array([+1.0, 0.0, 1.0]), {"roll":  0, "pitch": -1, "yaw": 0}),
    ("west   (drone -X of goal)", np.array([-1.0, 0.0, 1.0]), {"roll":  0, "pitch": +1, "yaw": 0}),
    ("north  (drone +Y of goal)", np.array([0.0, +1.0, 1.0]), {"roll": +1, "pitch":  0, "yaw": 0}),
    ("south  (drone -Y of goal)", np.array([0.0, -1.0, 1.0]), {"roll": -1, "pitch":  0, "yaw": 0}),
    ("on-target (drone == goal)", np.array([0.0, 0.0, 1.0]), {"roll":  0, "pitch":  0, "yaw": 0}),
]

# Pass/fail thresholds (deg/s).
ZERO_TOL_DEG_S = 50.0     # |rate| < this counts as "near zero"
ACTIVE_MIN_DEG_S = 20.0   # |rate| > this counts as "clearly non-zero"
SATURATION_DEG_S = (1.0 / np.pi * 180.0 * ANG_COMMAND_SCALE) * 0.99 # |rate| >= this is flagged as rail-saturated


def build_state(pos: np.ndarray, vz: float = _HOVER_VZ) -> np.ndarray:
    """13-vec at `pos`, with vertical velocity `vz`, identity attitude (xyzw),
    zero body rates."""
    state = np.zeros(13, dtype=np.float64)
    state[0:3] = pos
    state[5] = vz       # world-frame vz
    state[9] = 1.0      # quat w (xyzw)
    return state


def parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser(description=__doc__.splitlines()[1])
    p.add_argument(
        "policy_path",
        help="Path to the policy file (relative to src/rl/rl/, or absolute).",
    )
    p.add_argument(
        "--dump-obs",
        action="store_true",
        help="Per case, print the 12-element observation fed to the policy "
             "and the 4-element raw policy output (pre FRD-flip, pre rad→deg, "
             "pre thrust mapping). Useful for spotting obs-layout mismatches "
             "or saturated policy outputs.",
    )
    return p.parse_args()


def resolve_policy_path(arg: str) -> Path:
    candidate = Path(arg)
    if not candidate.is_absolute():
        candidate = _PKG_DIR / candidate
    if not candidate.exists():
        sys.exit(f"Policy not found: {candidate}")
    return candidate.resolve()


def check_axis(name: str, value: float, expected_sign: int) -> tuple[bool, str]:
    """Return (passed, reason). One-line verdict for a single axis."""
    if abs(value) >= SATURATION_DEG_S:
        return False, f"{name} saturated ({value:+.1f} deg/s, |.| > {SATURATION_DEG_S:.0f})"
    if expected_sign == 0:
        if abs(value) < ZERO_TOL_DEG_S:
            return True, ""
        return False, f"{name} should be ~0 but is {value:+.1f} deg/s"
    # expected non-zero, with sign
    if abs(value) < ACTIVE_MIN_DEG_S:
        return False, f"{name} too small ({value:+.1f} deg/s, want |.| > {ACTIVE_MIN_DEG_S:.0f} with sign {expected_sign:+d})"
    if (value > 0) != (expected_sign > 0):
        return False, f"{name} wrong sign ({value:+.1f} deg/s, want sign {expected_sign:+d})"
    return True, ""


def main() -> None:
    args = parse_args()
    policy_path = resolve_policy_path(args.policy_path)
    print(f"Loading policy: {policy_path}\n")

    # Override the hardcoded default so DroneRLController loads our policy.
    drone_rl_controller._POLICY_PATH = policy_path
    controller = drone_rl_controller.DroneRLController()

    print(f"All cases seeded with vz = {_HOVER_VZ:+.2f} m/s (world frame).")
    print(f"Tolerances: zero<{ZERO_TOL_DEG_S:.0f}  active>{ACTIVE_MIN_DEG_S:.0f}  "
          f"saturation>{SATURATION_DEG_S:.0f} deg/s\n")
    header = (f"{'case':<30} {'roll[deg/s]':>12} {'pitch[deg/s]':>13} "
              f"{'yaw[deg/s]':>11} {'thrust':>8}  verdict")
    print(header)
    print("-" * len(header))

    results = []  # (label, roll, pitch, yaw, thrust, ok, reasons)

    for label, pos, expect in CASES:
        state = build_state(pos)

        if args.dump_obs:
            import torch  # local: only needed for the debug path
            obs = controller.experiment.build_observation(state, GOAL)
            raw = controller.experiment.policy.inference(
                torch.tensor(obs, dtype=torch.float32).unsqueeze(0)
            ).squeeze(0).cpu().numpy()
            print(f"\n[{label}]")
            print(f"  obs  [gravity(3) ang_vel(3) waypoint/10(3) lin_vel(3)] = "
                  f"{np.array2string(obs, precision=4, separator=',')}")
            print(f"  raw  [thrust_axis roll_axis pitch_axis yaw_axis]     = "
                  f"{np.array2string(raw,  precision=4, separator=',')}")

        cmd = controller.controller_update(state, GOAL)

        roll_ok, roll_msg = check_axis("roll", cmd.roll_deg_s, expect["roll"])
        pitch_ok, pitch_msg = check_axis("pitch", cmd.pitch_deg_s, expect["pitch"])
        yaw_ok, yaw_msg = check_axis("yaw", cmd.yaw_deg_s, expect["yaw"])
        ok = roll_ok and pitch_ok and yaw_ok
        reasons = [m for m in (roll_msg, pitch_msg, yaw_msg) if m]

        verdict = "PASS" if ok else "FAIL"
        print(
            f"{label:<30} "
            f"{cmd.roll_deg_s:>12.4f} "
            f"{cmd.pitch_deg_s:>13.4f} "
            f"{cmd.yaw_deg_s:>11.4f} "
            f"{cmd.thrust_value:>8.4f}  {verdict}"
        )
        for r in reasons:
            print(f"  - {r}")

        results.append((label, cmd.roll_deg_s, cmd.pitch_deg_s, cmd.yaw_deg_s,
                        cmd.thrust_value, ok))

    # --- Cross-case symmetry checks --------------------------------------
    print("\nSymmetry checks (opposing cases should be near-mirrors):")
    by_label = {r[0]: r for r in results}
    sym_ok = True
    for a_key, b_key, axis_idx, axis_name in [
        ("east   (drone +X of goal)", "west   (drone -X of goal)", 2, "pitch"),
        ("north  (drone +Y of goal)", "south  (drone -Y of goal)", 1, "roll"),
    ]:
        a, b = by_label[a_key], by_label[b_key]
        va, vb = a[axis_idx], b[axis_idx]
        # Expect va ≈ -vb. Tolerance: 30% of mean magnitude, floor of 20 deg/s.
        mean_mag = (abs(va) + abs(vb)) / 2.0
        tol = max(0.3 * mean_mag, 20.0)
        residual = abs(va + vb)
        ok = residual < tol
        sym_ok &= ok
        verdict = "PASS" if ok else "FAIL"
        print(f"  {axis_name:<6}  {a_key.split()[0]:>6} {va:+8.2f}  vs  "
              f"{b_key.split()[0]:>6} {vb:+8.2f}  → residual {residual:.2f} "
              f"(tol {tol:.2f})  {verdict}")

    # --- On-target should not have the largest thrust ---------------------
    on_target = by_label["on-target (drone == goal)"]
    directional_max_thrust = max(by_label[k][4] for k in by_label if k != on_target[0])
    thrust_ok = on_target[4] <= directional_max_thrust + 1e-6
    print(f"\nThrust sanity: on-target thrust ({on_target[4]:.4f}) should be "
          f"≤ max directional thrust ({directional_max_thrust:.4f})  "
          f"→ {'PASS' if thrust_ok else 'FAIL'}")

    # --- Final verdict ----------------------------------------------------
    per_case_ok = all(r[5] for r in results)
    overall_ok = per_case_ok and sym_ok and thrust_ok
    print("\n" + ("ALL CHECKS PASSED" if overall_ok else "ONE OR MORE CHECKS FAILED"))
    sys.exit(0 if overall_ok else 1)


if __name__ == "__main__":
    main()
