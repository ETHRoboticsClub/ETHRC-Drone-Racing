"""
Generate synthetic labels (with outer corners) for frames where all 4 inner
corners are visible. Copies image + label into:

  synthetic_labels/train/<stem>.jpg  + <stem>.txt
  synthetic_labels/val/<stem>.jpg    + <stem>.txt

Re-running is safe: existing files are kept and only missing quota is filled.

Usage:
    python generate_labels.py               # 400 train + 100 val
    python generate_labels.py --train 400 --val 100 --scale 1.4
"""

import argparse
import glob
import os
import random
import shutil

import numpy as np

_GATE_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))

SCALE     = 1.4
DATA_ROOT = os.path.join(_GATE_ROOT, "data")
OUT_ROOT  = os.path.join(_GATE_ROOT, "data", "synthetic")


# ---------------------------------------------------------------------------
# Flight split (same seed as model.py)
# ---------------------------------------------------------------------------
def flight_dirs(split: str) -> list[str]:
    all_dirs = []
    for subdir in (os.path.join("raw", "autonomous"), "piloted"):
        all_dirs.extend(sorted(glob.glob(os.path.join(DATA_ROOT, subdir, "flight-*"))))
    random.seed(42)
    shuffled = all_dirs[:]
    random.shuffle(shuffled)
    cut = int(0.8 * len(shuffled))
    return shuffled[:cut] if split == "train" else shuffled[cut:]


# ---------------------------------------------------------------------------
# Label helpers
# ---------------------------------------------------------------------------
def all4_visible(label_path: str) -> bool:
    with open(label_path) as f:
        lines = [l for l in f if l.strip()]
    if not lines:
        return False
    for line in lines:
        vals = list(map(float, line.split()))
        if len(vals) < 17:
            return False
        vis = [vals[7], vals[10], vals[13], vals[16]]
        if not all(v > 0 for v in vis):
            return False
    return True


def make_synthetic_label(lbl_path: str, scale: float) -> list[str]:
    lines_out = []
    with open(lbl_path) as f:
        for line in f:
            vals = list(map(float, line.split()))
            if len(vals) < 17:
                continue
            cls = int(vals[0])
            cx, cy, w, h = vals[1:5]
            centroid = np.array([cx, cy], dtype=np.float32)

            inner_norm, vis_flags = [], []
            for k in range(4):
                b = 5 + k * 3
                inner_norm.append([vals[b], vals[b+1]])
                vis_flags.append(vals[b+2] > 0)

            original = f"{cls} {cx} {cy} {w} {h}"
            for k in range(4):
                original += f" {vals[5+k*3]} {vals[6+k*3]} {int(vals[7+k*3])}"

            outer_part = ""
            for (vx, vy), v_inner in zip(inner_norm, vis_flags):
                pt  = np.array([vx, vy], dtype=np.float32)
                opt = centroid + scale * (pt - centroid)
                ox, oy = float(opt[0]), float(opt[1])
                in_bounds = 0.0 <= ox <= 1.0 and 0.0 <= oy <= 1.0
                vis_out   = 2 if (v_inner and in_bounds) else 0
                outer_part += f" {ox:.10f} {oy:.10f} {vis_out}" if vis_out else " 0.0 0.0 0"

            lines_out.append(original + outer_part)
    return lines_out


def collect_candidates(dirs: list[str]) -> list[tuple[str, str]]:
    out = []
    for fdir in dirs:
        flight_name = os.path.basename(fdir)
        lbl_dir = os.path.join(fdir, f"label_{flight_name}")
        cam_dir = os.path.join(fdir, f"camera_{flight_name}")
        if not os.path.isdir(lbl_dir):
            continue
        for lbl in sorted(glob.glob(os.path.join(lbl_dir, "*.txt"))):
            if not all4_visible(lbl):
                continue
            stem = os.path.splitext(os.path.basename(lbl))[0]
            img  = os.path.join(cam_dir, stem + ".jpg")
            if os.path.exists(img):
                out.append((img, lbl))
    return out


# ---------------------------------------------------------------------------
# Main — top-up logic
# ---------------------------------------------------------------------------
def topup(split: str, target: int, scale: float):
    out_dir = os.path.join(OUT_ROOT, split)
    os.makedirs(out_dir, exist_ok=True)

    # Stems already present
    existing = {os.path.splitext(f)[0]
                for f in os.listdir(out_dir) if f.endswith(".txt")}
    have = len(existing)
    need = target - have

    if need <= 0:
        print(f"  {split}: already have {have}/{target}, nothing to do.")
        return

    print(f"  {split}: have {have}/{target}, need {need} more.")

    candidates = collect_candidates(flight_dirs(split))
    # Exclude already selected stems
    candidates = [(img, lbl) for img, lbl in candidates
                  if os.path.splitext(os.path.basename(lbl))[0] not in existing]

    random.seed(len(existing))   # different seed each top-up round
    selected = random.sample(candidates, min(need, len(candidates)))

    for img_path, lbl_path in selected:
        stem      = os.path.splitext(os.path.basename(lbl_path))[0]
        syn_lines = make_synthetic_label(lbl_path, scale)

        with open(os.path.join(out_dir, stem + ".txt"), "w") as f:
            f.write("\n".join(syn_lines) + "\n")
        shutil.copy2(img_path, os.path.join(out_dir, stem + ".jpg"))

    print(f"  {split}: added {len(selected)} samples → total {have + len(selected)}")


def main(scale: float, n_train: int, n_val: int):
    topup("train", n_train, scale)
    topup("val",   n_val,   scale)
    total = sum(
        len([f for f in os.listdir(os.path.join(OUT_ROOT, s)) if f.endswith(".txt")])
        for s in ("train", "val") if os.path.isdir(os.path.join(OUT_ROOT, s))
    )
    print(f"\nDone. {total} samples in {OUT_ROOT}/")


if __name__ == "__main__":
    p = argparse.ArgumentParser()
    p.add_argument("--scale", type=float, default=SCALE)
    p.add_argument("--train", type=int,   default=400)
    p.add_argument("--val",   type=int,   default=100)
    args = p.parse_args()
    main(args.scale, args.train, args.val)
