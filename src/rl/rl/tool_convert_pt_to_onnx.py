#!/usr/bin/env python3
"""Convert a policy .pt checkpoint to ONNX.

Usage:
    python tool_convert_pt_to_onnx.py hovering_policy.pt
    python tool_convert_pt_to_onnx.py hovering_policy.pt --opset 17

Reads from  models/pytorch/<name>.pt
Writes to   models/onnx/<name>.onnx
"""

import argparse
import sys
from pathlib import Path

import numpy as np
import torch
import torch.onnx

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))
from policy_model import PolicyModel
import hovering_experiment

_MODELS_DIR = Path(__file__).resolve().parent / "models"


def parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser(description="Export policy .pt → ONNX")
    p.add_argument("input", help="Filename inside models/pytorch/ (e.g. hovering_policy.pt)")
    p.add_argument("--obs-dim", type=int, default=hovering_experiment.OBS_DIM,
                   help=f"Observation dimension (default: {hovering_experiment.OBS_DIM})")
    p.add_argument("--opset", type=int, default=17,
                   help="ONNX opset version (default: 17)")
    return p.parse_args()


def export(args: argparse.Namespace) -> None:
    in_path = _MODELS_DIR / "pytorch" / args.input
    out_path = _MODELS_DIR / "onnx" / Path(args.input).with_suffix(".onnx").name
    out_path.parent.mkdir(parents=True, exist_ok=True)

    print(f"Loading checkpoint: {in_path}")
    model = PolicyModel(str(in_path))
    model.to(torch.device("cpu"))
    model.policy.eval()

    if model.running_mean is not None:
        obs_dim = model.running_mean.shape[0]
        print(f"Inferred obs_dim={obs_dim} from RunningStandardScaler weights.")
    else:
        obs_dim = args.obs_dim
        print(f"Using obs_dim={obs_dim} (no preprocessor found in checkpoint).")

    dummy = torch.zeros(1, obs_dim, dtype=torch.float32)

    class _WrappedPolicy(torch.nn.Module):
        def __init__(self, pm: PolicyModel):
            super().__init__()
            self.net = pm.policy
            self.running_mean = pm.running_mean.float() if pm.running_mean is not None else None
            self.running_var = pm.running_var.float() if pm.running_var is not None else None

        def forward(self, obs: torch.Tensor) -> torch.Tensor:
            if self.running_mean is not None and self.running_var is not None:
                std = torch.sqrt(self.running_var)
                std = torch.where(std < 1e-8, torch.ones_like(std), std)
                obs = (obs - self.running_mean) / std
            return self.net(obs)

    wrapped = _WrappedPolicy(model)
    wrapped.eval()

    torch.onnx.export(
        wrapped,
        dummy,
        str(out_path),
        input_names=["obs"],
        output_names=["actions"],
        dynamic_axes={"obs": {0: "batch"}, "actions": {0: "batch"}},
        opset_version=args.opset,
    )
    print(f"Exported → {out_path}  (opset {args.opset})")

    _verify(wrapped, dummy, str(out_path))


def _verify(torch_model: torch.nn.Module, dummy: torch.Tensor, onnx_path: str) -> None:
    try:
        import onnxruntime as ort
    except (ImportError, AttributeError) as e:
        print(f"[verify] onnxruntime unavailable ({e}) — skipping verification.")
        return

    with torch.no_grad():
        torch_out = torch_model(dummy).cpu().numpy()

    sess = ort.InferenceSession(onnx_path, providers=["CPUExecutionProvider"])
    onnx_out = sess.run(["actions"], {"obs": dummy.cpu().numpy()})[0]

    max_diff = float(np.abs(torch_out - onnx_out).max())
    print(f"[verify] Max output difference (torch vs onnx): {max_diff:.2e}")
    if max_diff > 1e-4:
        print("[verify] WARNING: outputs differ by more than 1e-4 — check preprocessing.")
    else:
        print("[verify] OK")


if __name__ == "__main__":
    export(parse_args())
