"""Slim port of DroneRacing_GateDetection's models package.

Drops the legacy ``GateNet`` (U-Net) backbone — the v3.4 checkpoint is
DeepLabV3+, the only architecture wired into ``build_model`` here.
"""

from .factory import build_model
from .interface import ModelConfig, SegmentationModelWrapper

__all__ = ["ModelConfig", "SegmentationModelWrapper", "build_model"]
