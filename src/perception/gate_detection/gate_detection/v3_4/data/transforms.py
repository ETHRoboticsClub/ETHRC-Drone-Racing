"""ImageNet normalisation helper — slim port of
DroneRacing_GateDetection/src/gate_detection/data/transforms.py.

The v3.4 checkpoint was trained with ``data.imagenet_norm: true``
(``configs/deeplabv3plus_keypoint_multi_balanced_v3_nextgate.yaml:20``),
so the predictor MUST apply the same mean/std at inference. Only the
constants + :func:`apply_imagenet_norm` are needed; the heavy
augmentation primitives are training-only.
"""

from __future__ import annotations

import numpy as np

# ImageNet-1k preprocessing constants for ResNet-style encoders. Matches
# segmentation_models_pytorch's resnet34 + imagenet pretrained_settings
# (see segmentation_models_pytorch/encoders/_legacy_pretrained_settings.py):
# RGB inputs in [0, 1], per-channel mean/std subtracted.
IMAGENET_MEAN = np.array([0.485, 0.456, 0.406], dtype=np.float32)
IMAGENET_STD  = np.array([0.229, 0.224, 0.225], dtype=np.float32)


def apply_imagenet_norm(img_rgb_uint8: np.ndarray) -> np.ndarray:
    """Per-channel ImageNet mean/std normalisation.

    Input: ``(H, W, 3)`` RGB image, **uint8 in [0, 255]**. Output:
    ``(H, W, 3)`` float32, scaled to ``[0, 1]`` and mean/std-subtracted
    per channel.
    """
    if img_rgb_uint8.dtype != np.uint8:
        raise TypeError(
            f"apply_imagenet_norm expects uint8 input, got {img_rgb_uint8.dtype}"
        )
    x = img_rgb_uint8.astype(np.float32) / 255.0
    return (x - IMAGENET_MEAN) / IMAGENET_STD
