#!/usr/bin/env python3
"""Run a deterministic airborne smoke test of the Boreas Gazebo plant."""

from __future__ import annotations

import argparse
from dataclasses import dataclass
import json
import math
import os
from pathlib import Path
import signal
import socket
import struct
import subprocess
import sys
import threading
import time
from typing import Any

import yaml


ROOT = Path(__file__).resolve().parents[2]
WORLD = ROOT / "sim/gazebo/worlds/sitl_plant_validation.sdf"
PROFILE = ROOT / "config/airframes/boreas_4s.yaml"
MOTOR_PORT = 9002


@dataclass
class MotorSender:
    """Continuously publish a four-motor Betaflight servo packet."""

    command: list[float]

    def __post_init__(self) -> None:
        """Create synchronization state without starting network traffic."""
        self._stop = threading.Event()
        self._lock = threading.Lock()
        self._thread = threading.Thread(target=self._run, daemon=True)

    def start(self) -> None:
        """Start publishing the current motor command."""
        self._thread.start()

    def set(self, command: list[float]) -> None:
        """Replace the four-motor command atomically."""
        with self._lock:
            self.command = list(command)

    def stop(self) -> None:
        """Stop the publisher and wait briefly for its thread."""
        self._stop.set()
        self._thread.join(timeout=1.0)

    def _run(self) -> None:
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        try:
            while not self._stop.is_set():
                with self._lock:
                    payload = struct.pack("<4f", *self.command)
                sock.sendto(payload, ("127.0.0.1", MOTOR_PORT))
                self._stop.wait(0.004)
        finally:
            sock.close()


def _gz_json(topic: str, timeout: float = 5.0) -> dict[str, Any]:
    result = subprocess.run(
        ["gz", "topic", "-e", "-n", "1", "--json-output", "-t", topic],
        check=True,
        capture_output=True,
        text=True,
        timeout=timeout,
    )
    # Some Gazebo releases emit a second sample despite ``-n 1``. Decode the
    # first complete JSON object instead of rejecting otherwise valid output.
    payload = result.stdout.lstrip()
    value, _ = json.JSONDecoder().raw_decode(payload)
    return value


def _wait_for_topic(topic: str, timeout: float) -> None:
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        result = subprocess.run(
            ["gz", "topic", "-l"], capture_output=True, text=True, check=False
        )
        if topic in result.stdout.splitlines():
            return
        time.sleep(0.2)
    raise TimeoutError(f"Gazebo topic did not appear: {topic}")


def _run_world() -> None:
    result = subprocess.run(
        [
            "gz",
            "service",
            "-s",
            "/world/sitl_plant_validation/control",
            "--reqtype",
            "gz.msgs.WorldControl",
            "--reptype",
            "gz.msgs.Boolean",
            "--timeout",
            "3000",
            "--req",
            "pause: false",
        ],
        capture_output=True,
        text=True,
        timeout=5.0,
    )
    if result.returncode != 0 or "data: true" not in result.stdout:
        raise RuntimeError(f"Could not start validation world: {result.stdout}{result.stderr}")


def _git_revision() -> str:
    result = subprocess.run(
        ["git", "rev-parse", "HEAD"], cwd=ROOT, capture_output=True, text=True
    )
    return result.stdout.strip() if result.returncode == 0 else "unknown"


def _command_for_per_motor_thrust(
    thrust_curve_n: list[float], target_thrust_n: float
) -> float:
    """Invert a0 + a1*u + a2*u^2 for a physical command in [0, 1]."""
    a0, a1, a2 = (float(value) for value in thrust_curve_n)
    if abs(a2) < 1e-12:
        command = (target_thrust_n - a0) / a1
    else:
        discriminant = max(a1 * a1 - 4.0 * a2 * (a0 - target_thrust_n), 0.0)
        roots = (
            (-a1 + math.sqrt(discriminant)) / (2.0 * a2),
            (-a1 - math.sqrt(discriminant)) / (2.0 * a2),
        )
        valid = [root for root in roots if 0.0 <= root <= 1.0]
        if not valid:
            raise ValueError("No physical motor command produces hover thrust")
        command = valid[0]
    return min(max(command, 0.0), 1.0)


def run_validation(timeout: float) -> dict[str, Any]:
    """Launch Gazebo, apply collective steps, and return validation metrics."""
    profile = yaml.safe_load(PROFILE.read_text(encoding="utf-8"))
    environment = os.environ.copy()
    resource_path = str(ROOT / "sim/gazebo/models")
    plugin_path = str(ROOT / "install/sitl_gazebo_plugins/lib")
    environment["GZ_SIM_RESOURCE_PATH"] = os.pathsep.join(
        filter(None, [resource_path, environment.get("GZ_SIM_RESOURCE_PATH")])
    )
    environment["GZ_SIM_SYSTEM_PLUGIN_PATH"] = os.pathsep.join(
        filter(None, [plugin_path, environment.get("GZ_SIM_SYSTEM_PLUGIN_PATH")])
    )
    environment["ETHRC_BOREAS_CONFIG"] = str(PROFILE)

    expected_hover_per_motor = (
        profile["rigid_body"]["mass_kg"] * 9.80665 / 4.0
    )
    hover_command = _command_for_per_motor_thrust(
        profile["propulsion"]["thrust_curve_n"], expected_hover_per_motor
    )
    sender = MotorSender([hover_command] * 4)
    process = subprocess.Popen(
        ["gz", "sim", str(WORLD), "-s", "--force-version", "8"],
        cwd=ROOT,
        env=environment,
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        text=True,
        start_new_session=True,
    )
    sender.start()
    try:
        _wait_for_topic("/drone/sim/motor_thrust", timeout)
        _wait_for_topic("/drone/odometry", timeout)
        _run_world()
        time.sleep(0.15)

        hover = _gz_json("/drone/odometry")
        thrust = _gz_json("/drone/sim/motor_thrust")
        hover_wrench = _gz_json("/drone/sim/applied_wrench")
        hover_z = float(hover["pose"]["position"]["z"])

        # Removing hover thrust must drive the actuator diagnostics to zero.
        sender.set([0.0, 0.0, 0.0, 0.0])
        time.sleep(0.15)
        freefall = _gz_json("/drone/odometry")
        freefall_thrust = _gz_json("/drone/sim/motor_thrust")
        freefall_z = float(freefall["pose"]["position"]["z"])
        freefall_velocity_z = float(
            freefall.get("twist", {}).get("linear", {}).get("z", 0.0)
        )
        stats = _gz_json("/world/sitl_plant_validation/stats")

        thrust_values = [float(value) for value in thrust["data"]]
        maximum_thrust_error = max(
            abs(value - expected_hover_per_motor) for value in thrust_values
        )
        freefall_thrust_values = [float(value) for value in freefall_thrust["data"]]
        applied_hover_force = [
            float(hover_wrench["force"].get(axis, 0.0)) for axis in ("x", "y", "z")
        ]
        applied_hover_force_magnitude = math.sqrt(
            sum(component * component for component in applied_hover_force)
        )
        expected_weight = profile["rigid_body"]["mass_kg"] * 9.80665
        passed = (
            maximum_thrust_error < 0.08
            and abs(applied_hover_force_magnitude - expected_weight) < 0.20
            and max(freefall_thrust_values) < 0.02
        )

        return {
            "passed": passed,
            "airframe": profile["airframe"],
            "calibration_status": profile["calibration_status"],
            "git_revision": _git_revision(),
            "physics_step_s": 0.001,
            "hover_altitude_m": hover_z,
            "freefall_altitude_m": freefall_z,
            "freefall_velocity_z_m_s": freefall_velocity_z,
            "freefall_thrust_per_motor_n": freefall_thrust_values,
            "applied_hover_force_world_n": applied_hover_force,
            "applied_hover_force_magnitude_n": applied_hover_force_magnitude,
            "expected_weight_n": expected_weight,
            "hover_thrust_per_motor_n": thrust_values,
            "expected_hover_thrust_per_motor_n": expected_hover_per_motor,
            "hover_motor_command": hover_command,
            "maximum_hover_thrust_error_n": maximum_thrust_error,
            "real_time_factor": float(stats.get("realTimeFactor", 0.0)),
        }
    finally:
        sender.stop()
        if process.poll() is None:
            os.killpg(process.pid, signal.SIGTERM)
            try:
                process.wait(timeout=5.0)
            except subprocess.TimeoutExpired:
                os.killpg(process.pid, signal.SIGKILL)
                process.wait(timeout=2.0)


def main() -> int:
    """Run the plant validation and optionally save its JSON report."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--timeout", type=float, default=20.0)
    parser.add_argument("--report", type=Path)
    args = parser.parse_args()

    try:
        report = run_validation(args.timeout)
    except (KeyError, OSError, RuntimeError, TimeoutError, ValueError) as error:
        print(f"SITL plant validation FAILED: {error}", file=sys.stderr)
        return 1

    rendered = json.dumps(report, indent=2, sort_keys=True)
    print(rendered)
    if args.report:
        args.report.write_text(rendered + "\n", encoding="utf-8")
    return 0 if report["passed"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
