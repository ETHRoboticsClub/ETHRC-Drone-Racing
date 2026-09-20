#!/usr/bin/env python3
"""Compare Isaac raw policy I/O with the equivalent Betaflight SITL trace."""

from __future__ import annotations

import argparse
import json
from pathlib import Path

import numpy as np


def _read(path: Path) -> list[dict]:
    return [json.loads(line) for line in path.read_text().splitlines() if line.strip()]


def _sitl_raw_action(command: list[float]) -> np.ndarray:
    """Undo the Boreas deployment adapter (500 deg/s, yaw sign, quadratic thrust)."""
    roll, pitch, yaw, throttle = np.asarray(command, dtype=float)
    return np.array([
        2.0 * np.clip(throttle, 0.0, 1.0) ** 2 - 1.0,
        roll / 500.0,
        pitch / 500.0,
        -yaw / 500.0,
    ])


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("isaac", type=Path)
    parser.add_argument("sitl", type=Path)
    parser.add_argument("--observation-tol", type=float, default=2e-5)
    parser.add_argument("--action-tol", type=float, default=2e-4)
    parser.add_argument("--rows", type=int, default=1, help="Compare this many synchronized identical-state rows")
    args = parser.parse_args()

    isaac, sitl = _read(args.isaac), _read(args.sitl)
    count = min(len(isaac), len(sitl), args.rows)
    if count == 0:
        raise SystemExit("both traces must contain at least one row")
    obs_err, act_err = [], []
    for i in range(count):
        obs_err.append(np.max(np.abs(np.asarray(isaac[i]["observation"]) - np.asarray(sitl[i]["observation"]))))
        act_err.append(np.max(np.abs(np.asarray(isaac[i]["action"]) - _sitl_raw_action(sitl[i]["command"]))))
    report = {
        "rows_compared": count,
        "observation_max_abs_error": float(max(obs_err)),
        "action_max_abs_error": float(max(act_err)),
        "observation_passed": bool(max(obs_err) <= args.observation_tol),
        "action_passed": bool(max(act_err) <= args.action_tol),
    }
    report["passed"] = report["observation_passed"] and report["action_passed"]
    print(json.dumps(report, indent=2, sort_keys=True))
    return 0 if report["passed"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
