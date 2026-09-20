#!/usr/bin/env python3
"""Validate the static Phase-1 Gazebo perception ingress contract.

This is intentionally a preflight check. Runtime topic-rate/stamp validation
will be added with the sensor adapter, when the production mono/RGB contract
can be produced without pretending that the current Gazebo camera is calibrated.
"""

from __future__ import annotations

import argparse
from pathlib import Path
import sys

from perception_contract import format_errors, validate_contract


def main() -> int:
    root = Path(__file__).resolve().parents[2]
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--bridge-config", type=Path, default=root / "src/utils/gazebo_ros_bridges/config/gz_bridges.yaml")
    parser.add_argument("--model-sdf", type=Path, default=root / "sim/gazebo/models/drone_sitl/model.sdf")
    args = parser.parse_args()

    errors = validate_contract(args.bridge_config, args.model_sdf)
    if errors:
        print("SITL perception ingress contract failed:")
        print(format_errors(errors))
        return 1
    print("SITL perception ingress static contract passed.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
