import argparse
import sys
import time
from pathlib import Path

import numpy as np
import torch

from rl.policy_model import PolicyModel
from rl import hovering_experiment

_MODELS_DIR = Path(__file__).resolve().parent / "models"


def parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser(description="Test PyTorch and ONNX policy outputs")
    p.add_argument("name", help="Model name (e.g. hovering_policy)")
    return p.parse_args()


def test(name: str) -> None:
    stem = Path(name).stem
    pt_path = _MODELS_DIR / "pytorch" / f"{stem}.pt"
    onnx_path = _MODELS_DIR / "onnx" / f"{stem}.onnx"

    # --- PyTorch ---
    print(f"Loading PyTorch: {pt_path}")
    model = PolicyModel(model_path=str(pt_path), model_type="pytorch")
    model.to(torch.device("cpu"))
    model.policy.eval()

    obs_dim = model.running_mean.shape[0] if model.running_mean is not None else hovering_experiment.OBS_DIM
    rng = np.random.default_rng(0)
    obs_np = rng.standard_normal((4, obs_dim)).astype(np.float32)
    obs_t = torch.tensor(obs_np)

    N = 200
    with torch.no_grad():
        for _ in range(20):  # warmup
            model.inference(obs_t)
        t0 = time.perf_counter()
        for _ in range(N):
            pt_out = model.inference(obs_t)
        pt_ms = (time.perf_counter() - t0) / N * 1000
    pt_out = pt_out.numpy()

    print(f"  obs shape : {obs_np.shape}")
    print(f"  output    : {pt_out}")
    print(f"  latency   : {pt_ms:.3f} ms/call (avg over {N})")

    # --- ONNX ---
    if not onnx_path.exists():
        print(f"\nONNX model not found at {onnx_path} — skipping ONNX test.")
        print("Run tool_convert_pt_to_onnx.py first.")
        return

    print(f"\nLoading ONNX: {onnx_path}")
    try:
        onnx_model = PolicyModel(model_path=str(onnx_path), model_type="onnx")
    except RuntimeError as e:
        print(f"  skipping: {e}")
        return

    for _ in range(20):  # warmup
        onnx_model.inference(obs_t)
    t0 = time.perf_counter()
    for _ in range(N):
        onnx_out = onnx_model.inference(obs_t)
    onnx_ms = (time.perf_counter() - t0) / N * 1000
    onnx_out = onnx_out.numpy()
    print(f"  output    : {onnx_out}")
    print(f"  latency   : {onnx_ms:.3f} ms/call (avg over {N})")

    # --- Compare ---
    max_diff = float(np.abs(pt_out - onnx_out).max())
    mean_diff = float(np.abs(pt_out - onnx_out).mean())
    print(f"\nMax diff  : {max_diff:.2e}")
    print(f"Mean diff : {mean_diff:.2e}")
    print(f"Speedup   : {pt_ms / onnx_ms:.2f}x  (PyTorch {pt_ms:.3f} ms vs ONNX {onnx_ms:.3f} ms)")
    if max_diff < 1e-4:
        print("PASS — outputs match.")
    else:
        print("FAIL — outputs differ beyond tolerance.")
        sys.exit(1)


if __name__ == "__main__":
    test(parse_args().name)
