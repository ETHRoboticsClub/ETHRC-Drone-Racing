"""Model factory — slim port (deeplabv3plus branch only).

The upstream factory also dispatches to ``gatenet``; that branch was
dropped because the v3.4 checkpoint is DeepLabV3+ and the legacy
GateNet backbone (with its ``components.DoubleConv/Down/Up/OutConv``
dep) is not used by this ROS node. An unsupported ``cfg.name`` raises
``ValueError`` — CLAUDE.md §6, no silent fallback.
"""

from __future__ import annotations

from .deeplabv3plus import build_deeplabv3plus
from .interface import ModelConfig, SegmentationModelWrapper


def build_model(cfg: ModelConfig) -> SegmentationModelWrapper:
    """Construct a wrapped segmentation model from :class:`ModelConfig`."""
    name = cfg.name.lower()
    if name == "deeplabv3plus":
        return build_deeplabv3plus(
            encoder_name=cfg.encoder_name,
            encoder_weights=cfg.encoder_weights,
            dropout_p=cfg.dropout_p,
            keypoint_head=cfg.keypoint_head,
            n_keypoints=cfg.n_keypoints,
            next_gate_head=cfg.next_gate_head,
        )
    raise ValueError(
        f"Unsupported model.name={cfg.name!r}; only 'deeplabv3plus' is "
        f"wired in the ROS port. Re-train with deeplabv3plus or extend "
        f"this factory in gate_detection/v3_4/models/factory.py."
    )
