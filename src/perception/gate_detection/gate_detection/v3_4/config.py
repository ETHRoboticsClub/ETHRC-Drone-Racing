"""Device selection helper — slim port of
DroneRacing_GateDetection/src/gate_detection/config.py.

Only ``get_device`` is needed by the v3.4 predictor; the YAML-loader
(``load_config``) and the path constants live upstream and are not
useful inside the ROS package.
"""

from __future__ import annotations


def get_device() -> str:
    """Pick the best available torch device (cuda > mps > cpu)."""
    import torch
    if torch.cuda.is_available():
        return "cuda"
    if torch.backends.mps.is_available():
        return "mps"
    return "cpu"
