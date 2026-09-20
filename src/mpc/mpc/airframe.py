"""Loading of the shared Boreas airframe model used by SITL and MPC."""

from __future__ import annotations

from dataclasses import dataclass
import os
from pathlib import Path

import yaml


@dataclass(frozen=True)
class BoreasAirframe:
    """Physical parameters required by the high-level MPC model."""

    mass_kg: float
    inertia_kg_m2: tuple[float, float, float]
    maximum_thrust_n: float
    thrust_curve_per_motor_n: tuple[float, float, float]
    translational_drag: tuple[float, float, float]
    calibration_status: str
    source_path: Path


def _candidate_paths() -> list[Path]:
    configured = os.environ.get("ETHRC_BOREAS_CONFIG")
    candidates = [Path(configured)] if configured else []
    candidates.append(Path.cwd() / "config/airframes/boreas_4s.yaml")
    for parent in Path(__file__).resolve().parents:
        candidates.append(parent / "config/airframes/boreas_4s.yaml")
    return candidates


def find_boreas_config() -> Path:
    """Find the canonical profile in a source workspace or configured path."""
    for candidate in _candidate_paths():
        if candidate.is_file():
            return candidate.resolve()
    searched = "\n  - ".join(str(path) for path in _candidate_paths())
    raise FileNotFoundError(
        "Cannot find config/airframes/boreas_4s.yaml. Set "
        f"ETHRC_BOREAS_CONFIG explicitly. Searched:\n  - {searched}"
    )


def load_boreas_airframe(path: Path | None = None) -> BoreasAirframe:
    """Load and validate the shared Boreas profile."""
    source_path = (path or find_boreas_config()).resolve()
    data = yaml.safe_load(source_path.read_text(encoding="utf-8"))
    if data.get("schema_version") != 1 or data.get("airframe") != "boreas_4s":
        raise ValueError(f"Unsupported airframe profile: {source_path}")

    rigid_body = data["rigid_body"]
    inertia = rigid_body["inertia_kg_m2"]
    thrust_curve = data["propulsion"]["thrust_curve_n"]
    if len(thrust_curve) != 3:
        raise ValueError("propulsion.thrust_curve_n must have three coefficients")
    per_motor_maximum = sum(float(value) for value in thrust_curve)

    return BoreasAirframe(
        mass_kg=float(rigid_body["mass_kg"]),
        inertia_kg_m2=(
            float(inertia["ixx"]),
            float(inertia["iyy"]),
            float(inertia["izz"]),
        ),
        maximum_thrust_n=4.0 * per_motor_maximum,
        thrust_curve_per_motor_n=tuple(float(value) for value in thrust_curve),
        translational_drag=tuple(
            float(value) for value in data["aerodynamics"]["translational_drag"]
        ),
        calibration_status=str(data["calibration_status"]),
        source_path=source_path,
    )
