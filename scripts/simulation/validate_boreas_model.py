#!/usr/bin/env python3
"""Validate that the Boreas profile, URDF, and Gazebo SDF agree."""

from __future__ import annotations

import argparse
import math
from pathlib import Path
import sys
import xml.etree.ElementTree as ET

import yaml


ROOT = Path(__file__).resolve().parents[2]
DEFAULT_CONFIG = ROOT / "config/airframes/boreas_4s.yaml"
DEFAULT_SDF = ROOT / "sim/gazebo/models/drone_sitl/model.sdf"


def _floats(text: str) -> list[float]:
    return [float(value) for value in text.split()]


def _assert_close(name: str, actual: float, expected: float, tolerance: float) -> None:
    if not math.isclose(actual, expected, rel_tol=0.0, abs_tol=tolerance):
        raise ValueError(f"{name}: {actual:.12g} != {expected:.12g}")


def _assert_vector(
    name: str,
    actual: list[float],
    expected: list[float],
    tolerance: float,
) -> None:
    if len(actual) != len(expected):
        raise ValueError(f"{name}: length {len(actual)} != {len(expected)}")
    for index, (actual_value, expected_value) in enumerate(zip(actual, expected)):
        _assert_close(f"{name}[{index}]", actual_value, expected_value, tolerance)


def validate(config_path: Path, sdf_path: Path, tolerance: float) -> None:
    """Raise ValueError when the three Boreas model representations diverge."""
    config = yaml.safe_load(config_path.read_text(encoding="utf-8"))
    if config.get("airframe") != "boreas_4s":
        raise ValueError("configuration airframe must be boreas_4s")

    rigid_body = config["rigid_body"]
    urdf_path = (config_path.parent / rigid_body["source_urdf"]).resolve()
    urdf_root = ET.parse(urdf_path).getroot()
    urdf_inertial = urdf_root.find("./link[@name='base_link']/inertial")
    if urdf_inertial is None:
        raise ValueError("URDF base_link/inertial is missing")

    urdf_mass = float(urdf_inertial.find("mass").attrib["value"])
    urdf_com = _floats(urdf_inertial.find("origin").attrib["xyz"])
    urdf_inertia = {
        key: float(value)
        for key, value in urdf_inertial.find("inertia").attrib.items()
    }

    _assert_close("config/URDF mass", rigid_body["mass_kg"], urdf_mass, tolerance)
    _assert_vector(
        "config/URDF center_of_mass",
        rigid_body["center_of_mass_m"],
        urdf_com,
        tolerance,
    )
    for key, value in rigid_body["inertia_kg_m2"].items():
        _assert_close(f"config/URDF inertia.{key}", value, urdf_inertia[key], tolerance)

    sdf_root = ET.parse(sdf_path).getroot()
    sdf_inertial = sdf_root.find("./model/link[@name='base_link']/inertial")
    if sdf_inertial is None:
        raise ValueError("SDF base_link/inertial is missing")
    _assert_close(
        "SDF/URDF mass", float(sdf_inertial.findtext("mass")), urdf_mass, tolerance
    )
    _assert_vector(
        "SDF/URDF center_of_mass",
        _floats(sdf_inertial.findtext("pose"))[:3],
        urdf_com,
        tolerance,
    )
    for key, value in urdf_inertia.items():
        _assert_close(
            f"SDF/URDF inertia.{key}",
            float(sdf_inertial.findtext(f"inertia/{key}")),
            value,
            tolerance,
        )

    sdf_rotors = {
        int(node.attrib["id"]): node
        for node in sdf_root.findall("./model/plugin/rotor")
    }
    config_rotors = config["rotors"]
    expected_motor_contract = {
        0: (1, "back_right", "cw"),
        1: (2, "front_right", "ccw"),
        2: (3, "back_left", "ccw"),
        3: (4, "front_left", "cw"),
    }
    if len(sdf_rotors) != len(config_rotors):
        raise ValueError("SDF and Boreas profile have different rotor counts")
    for rotor in config_rotors:
        expected = expected_motor_contract[rotor["id"]]
        actual = (
            rotor["betaflight_motor"],
            rotor["name"],
            rotor["turning_direction"],
        )
        if actual != expected:
            raise ValueError(
                f"rotor {rotor['id']} violates Betaflight motor contract: "
                f"{actual} != {expected}"
            )
        sdf_rotor = sdf_rotors[rotor["id"]]
        if sdf_rotor.findtext("jointName") != rotor["joint_name"]:
            raise ValueError(f"rotor {rotor['id']} joint name differs")
        if sdf_rotor.findtext("turningDirection") != rotor["turning_direction"]:
            raise ValueError(f"rotor {rotor['id']} turning direction differs")
        _assert_vector(
            f"rotor {rotor['id']} position",
            _floats(sdf_rotor.findtext("position")),
            rotor["position_m"],
            tolerance,
        )


def main() -> int:
    """Run the consistency check from command-line arguments."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", type=Path, default=DEFAULT_CONFIG)
    parser.add_argument("--sdf", type=Path, default=DEFAULT_SDF)
    parser.add_argument("--tolerance", type=float, default=1e-12)
    args = parser.parse_args()

    try:
        validate(args.config.resolve(), args.sdf.resolve(), args.tolerance)
    except (KeyError, OSError, TypeError, ValueError, ET.ParseError, yaml.YAMLError) as error:
        print(f"Boreas model validation FAILED: {error}", file=sys.stderr)
        return 1

    print("Boreas model validation passed: profile, URDF, and SDF agree.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
