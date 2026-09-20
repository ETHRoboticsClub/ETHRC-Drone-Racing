"""Slim ROS port of DroneRacing_GateDetection's ``inference.predictor``.

Keeps the full checkpoint-loading helpers (which handle
``_orig_mod.`` torch.compile prefixes, raw-vs-wrapped state_dicts, and
``model_config``/``data_config`` round-tripping — all load-bearing for
correct inference) but drops the disk-reading entry points
(``predict_image``, ``predict_keypoints``, ``predict_keypoint_heatmap``)
and the soft-argmax / coherent-quad lazy imports they used.

The single retained entry point for ROS use is
:meth:`GatePredictor.predict_full_from_bgr`, which mirrors upstream
``predict_full`` but takes a BGR ndarray (what ``cv_bridge`` returns)
instead of a filesystem path.
"""

from __future__ import annotations

import os
from pathlib import Path

import cv2
import numpy as np
import torch

from gate_detection.v3_4.config import get_device
from gate_detection.v3_4.models import (
    ModelConfig,
    SegmentationModelWrapper,
    build_model,
)


def _normalize_state_dict_keys(state_dict: dict[str, torch.Tensor]) -> dict[str, torch.Tensor]:
    """Drop torch.compile wrapper prefixes (e.g. ``_orig_mod.``) when present.

    The trainer already strips ``_orig_mod.`` via ``_base_model`` before
    saving, so this is mostly defensive — covers checkpoints saved by older
    versions of the trainer or by external tooling.
    """
    prefix = "_orig_mod."
    if any(k.startswith(prefix) for k in state_dict):
        return {
            (k[len(prefix):] if k.startswith(prefix) else k): v
            for k, v in state_dict.items()
        }
    return state_dict


def _load_state_dict_flexible(
    model: SegmentationModelWrapper,
    state_dict: dict[str, torch.Tensor],
) -> None:
    """Load either wrapped (model.*) or legacy raw-backbone checkpoints.

    ``SegmentationModelWrapper`` stores the actual backbone under ``self.model``,
    so wrapped state-dicts have a ``model.`` prefix on every key. Legacy
    GateNet checkpoints predate the wrapper and were saved as raw backbone
    state-dicts (keys like ``inc.net.0.weight``); detect by absence of the
    prefix and load directly into the inner backbone.
    """
    has_wrapper_prefix = any(k.startswith("model.") for k in state_dict)
    if has_wrapper_prefix:
        model.load_state_dict(state_dict)
        return

    # Legacy checkpoints saved raw backbone keys (e.g. "inc.*").
    model.model.load_state_dict(state_dict)


def _load_model_config(
    ckpt: dict,
    *,
    model_name: str,
    width_factor: int,
    encoder_name: str,
    encoder_weights: str | None,
    dropout_p: float,
    keypoint_head: bool,
    n_keypoints: int,
    next_gate_head: bool = False,
) -> ModelConfig:
    """Resolve model config from checkpoint first, then explicit args.

    The saved ``model_config`` is authoritative — every architectural field
    that affects the state-dict layout (including ``dropout_p``, which gates
    the dropout-wrapped segmentation head, and ``keypoint_head`` /
    ``n_keypoints`` / ``next_gate_head``, which add extra heads) must
    round-trip from it.
    """
    saved = ckpt.get("model_config", {}) if isinstance(ckpt, dict) else {}
    return ModelConfig(
        name=saved.get("name", model_name),
        width_factor=int(saved.get("width_factor", width_factor)),
        encoder_name=saved.get("encoder_name", encoder_name),
        encoder_weights=saved.get("encoder_weights", encoder_weights),
        dropout_p=float(saved.get("dropout_p", dropout_p)),
        keypoint_head=bool(saved.get("keypoint_head", keypoint_head)),
        n_keypoints=int(saved.get("n_keypoints", n_keypoints)),
        next_gate_head=bool(saved.get("next_gate_head", next_gate_head)),
    )


def load_checkpoint(
    path: str | os.PathLike,
    *,
    model_name: str = "gatenet",
    width_factor: int = 4,
    encoder_name: str = "resnet34",
    encoder_weights: str | None = None,
    dropout_p: float = 0.0,
    keypoint_head: bool = False,
    n_keypoints: int = 4,
    next_gate_head: bool = False,
    device: str | None = None,
) -> tuple[SegmentationModelWrapper, dict]:
    """Load a segmentation checkpoint. Returns ``(model.eval(), checkpoint_dict)``."""
    device = device or get_device()
    payload = torch.load(str(path), map_location=device, weights_only=True)

    # Two on-disk formats:
    #   * dict with metadata: {"model": state_dict, "epoch": ..., "model_config": ...}
    #   * raw state_dict (rare; e.g. third-party weights or torch.save(model.state_dict()))
    if isinstance(payload, dict) and "model" in payload:
        ckpt = payload
        state_dict = ckpt["model"]
    else:
        state_dict = payload
        ckpt = {"model": state_dict}

    model_cfg = _load_model_config(
        ckpt,
        model_name=model_name,
        width_factor=width_factor,
        encoder_name=encoder_name,
        encoder_weights=encoder_weights,
        dropout_p=dropout_p,
        keypoint_head=keypoint_head,
        n_keypoints=n_keypoints,
        next_gate_head=next_gate_head,
    )
    model = build_model(model_cfg).to(device)

    state_dict = _normalize_state_dict_keys(state_dict)
    ckpt["model"] = state_dict
    ckpt.setdefault("model_config", {
        "name": model_cfg.name,
        "width_factor": model_cfg.width_factor,
        "encoder_name": model_cfg.encoder_name,
        "encoder_weights": model_cfg.encoder_weights,
        "dropout_p": model_cfg.dropout_p,
        "keypoint_head": model_cfg.keypoint_head,
        "n_keypoints": model_cfg.n_keypoints,
        "next_gate_head": model_cfg.next_gate_head,
    })
    _load_state_dict_flexible(model, state_dict)
    model.eval()
    return model, ckpt


class GatePredictor:
    """Run a v3.4 DeepLabV3+ multi-head model on in-memory BGR frames.

    Single ROS entry point: :meth:`predict_full_from_bgr`. Returns the
    four tensors that :class:`HybridSegNextGateTracker` needs (resized
    RGB, seg prob, kp heatmap, next-gate mask).
    """

    def __init__(
        self,
        checkpoint: str | os.PathLike,
        *,
        img_size: int = 384,
        model_name: str = "gatenet",
        width_factor: int = 4,
        encoder_name: str = "resnet34",
        encoder_weights: str | None = None,
        dropout_p: float = 0.0,
        keypoint_head: bool = False,
        n_keypoints: int = 4,
        device: str | None = None,
    ):
        self.device = device or get_device()
        self.img_size = img_size
        self.model, self.checkpoint_meta = load_checkpoint(
            checkpoint,
            model_name=model_name,
            width_factor=width_factor,
            encoder_name=encoder_name,
            encoder_weights=encoder_weights,
            dropout_p=dropout_p,
            keypoint_head=keypoint_head,
            n_keypoints=n_keypoints,
            device=self.device,
        )
        # Replay the data preprocessing the checkpoint was trained with.
        # Old checkpoints (no data_config field) default to imagenet_norm=False
        # — same behaviour as before this flag was introduced. We log the
        # fallback explicitly per CLAUDE.md §6 (no silent defaults).
        if "data_config" not in self.checkpoint_meta:
            print(
                f"[WARN] checkpoint at {checkpoint}: no data_config; "
                "defaulting imagenet_norm=False (legacy behaviour)",
                flush=True,
            )
        data_cfg = self.checkpoint_meta.get("data_config", {}) or {}
        self.imagenet_norm: bool = bool(data_cfg.get("imagenet_norm", False))
        # Multi-gate target flag — checkpoints trained with multi-modal
        # heatmaps need windowed soft-argmax at decode (global soft-argmax
        # averages across multiple peaks per channel; see report 10).
        self.multi_gate_target: bool = bool(data_cfg.get("multi_gate_target", False))
        # Authoritative keypoint-head flag from the (now-loaded) model
        # config — checkpoint metadata wins over the ``keypoint_head`` arg.
        mcfg = self.checkpoint_meta.get("model_config", {}) or {}
        self.has_keypoint_head: bool = bool(mcfg.get("keypoint_head", keypoint_head))
        self.n_keypoints: int = int(mcfg.get("n_keypoints", n_keypoints))
        # Next-gate-mask head (report 15). False for any pre-report-15
        # checkpoint (the legacy v2 ckpt and earlier) — those models'
        # forward returns ng_pred=None and predict_full_from_bgr surfaces
        # None for the ng_mask field so downstream trackers can branch
        # cleanly.
        self.has_next_gate_head: bool = bool(mcfg.get("next_gate_head", False))

    @torch.no_grad()
    def predict_full_from_bgr(
        self,
        img_bgr: np.ndarray,
    ) -> tuple[np.ndarray, np.ndarray, np.ndarray | None, np.ndarray | None]:
        """Return ``(rgb_uint8, seg_prob, kp_heatmap_or_None, ng_mask_or_None)``.

        Mirror of upstream ``predict_full`` but accepts a BGR ndarray
        directly (what ``cv_bridge.imgmsg_to_cv2(desired_encoding='bgr8')``
        returns) so the hot path avoids a disk round-trip.
        """
        img_t, img_rgb = self._preprocess_bgr(img_bgr)
        if not self.has_keypoint_head:
            seg = self._predict_tensor(img_t).squeeze().cpu().numpy()
            return img_rgb, seg, None, None
        seg_tuple, kp_pred, ng_pred = self.model.forward_multi(img_t)
        seg = seg_tuple[-1].squeeze().cpu().numpy()
        kp_pred_np = kp_pred.detach().float().cpu().numpy()[0]
        ng_pred_np: np.ndarray | None = None
        if ng_pred is not None:
            # (B=1, 1, H, W) -> (H, W)
            ng_pred_np = ng_pred.detach().float().cpu().numpy()[0, 0]
        return img_rgb, seg, kp_pred_np, ng_pred_np

    @torch.no_grad()
    def _predict_tensor(self, img_t: torch.Tensor) -> torch.Tensor:
        """Forward pass returning the full-resolution probability map (NHW or NCHW)."""
        return self.model(img_t)[-1]

    def _preprocess_bgr(self, img_bgr: np.ndarray) -> tuple[torch.Tensor, np.ndarray]:
        """BGR ndarray → (NCHW tensor on device, RGB ``img_size×img_size`` uint8)."""
        img_rgb = cv2.cvtColor(img_bgr, cv2.COLOR_BGR2RGB)
        img_res = cv2.resize(img_rgb, (self.img_size, self.img_size))
        if self.imagenet_norm:
            # Match the training-time path in GateDataset.__getitem__ via
            # the single shared helper in data/transforms.py.
            from gate_detection.v3_4.data.transforms import apply_imagenet_norm
            img_t = torch.from_numpy(apply_imagenet_norm(img_res)).permute(2, 0, 1).contiguous()
        else:
            img_t = torch.from_numpy(img_res).permute(2, 0, 1).float() / 255.0
        return img_t.unsqueeze(0).to(self.device), img_res
