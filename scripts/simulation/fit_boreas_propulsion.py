#!/usr/bin/env python3
"""Fit the Boreas static thrust curve from thrust-stand CSV measurements."""

from __future__ import annotations

import argparse
import csv
import json
from pathlib import Path
import sys

import numpy as np


REQUIRED_COLUMNS = {"rpm", "thrust_n"}


def load_measurements(path: Path) -> tuple[np.ndarray, np.ndarray]:
    """Load finite positive RPM/thrust samples from a CSV file."""
    with path.open(newline="", encoding="utf-8") as handle:
        reader = csv.DictReader(handle)
        if not REQUIRED_COLUMNS.issubset(reader.fieldnames or []):
            raise ValueError(
                f"CSV requires columns {sorted(REQUIRED_COLUMNS)}; got {reader.fieldnames}"
            )
        rows = [
            (float(row["rpm"]), float(row["thrust_n"]))
            for row in reader
            if row["rpm"] and row["thrust_n"]
        ]
    values = np.asarray(rows, dtype=float)
    if values.shape[0] < 6 or not np.isfinite(values).all():
        raise ValueError("at least six finite RPM/thrust samples are required")
    keep = (values[:, 0] >= 0.0) & (values[:, 1] >= 0.0)
    return values[keep, 0], values[keep, 1]


def fit_curve(rpm: np.ndarray, thrust_n: np.ndarray, maximum_rpm: float) -> dict:
    """Fit T=c1*n+c2*n^2 with the physical constraint T(0)=0."""
    if maximum_rpm <= 0.0 or maximum_rpm < float(np.max(rpm)):
        raise ValueError("maximum-rpm must be positive and cover every measurement")
    normalized = rpm / maximum_rpm
    design = np.column_stack((normalized, normalized * normalized))
    coefficients, _, _, _ = np.linalg.lstsq(design, thrust_n, rcond=None)
    predicted = design @ coefficients
    residual = predicted - thrust_n
    rmse = float(np.sqrt(np.mean(residual * residual)))
    full_scale = float(coefficients.sum())
    return {
        "model": "normalized_rotor_speed_polynomial",
        "maximum_rpm": maximum_rpm,
        "thrust_curve_n": [0.0, float(coefficients[0]), float(coefficients[1])],
        "samples": int(rpm.size),
        "rmse_n": rmse,
        "rmse_percent_full_scale": 100.0 * rmse / full_scale,
        "maximum_thrust_per_motor_n": full_scale,
    }


def main() -> int:
    """Fit measurements and print or save the resulting report."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("input_csv", type=Path)
    parser.add_argument("--maximum-rpm", type=float, required=True)
    parser.add_argument("--report", type=Path)
    args = parser.parse_args()
    try:
        rpm, thrust = load_measurements(args.input_csv)
        report = fit_curve(rpm, thrust, args.maximum_rpm)
    except (OSError, TypeError, ValueError) as error:
        print(f"Boreas propulsion fit FAILED: {error}", file=sys.stderr)
        return 1

    rendered = json.dumps(report, indent=2, sort_keys=True)
    print(rendered)
    if args.report:
        args.report.write_text(rendered + "\n", encoding="utf-8")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
