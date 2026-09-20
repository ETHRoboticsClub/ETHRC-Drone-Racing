"""
Run inference and visualise all QuAdGate pipeline steps.

Layout per image row (7 columns):
  Image | GT mask | Pred mask | QAG: lines | QAG: candidates | QAG: matches | QAG: corners

Usage:
    python scripts/inference.py checkpoints/best_gatenet.pth
    python scripts/inference.py checkpoints/best_gatenet.pth --n 12 --out inference.png
    python scripts/inference.py checkpoints/best_gatenet.pth --images path/a.jpg path/b.jpg
"""

import argparse
import glob
import os
import random

_GATE_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
import sys
sys.path.insert(0, _GATE_ROOT)

import cv2
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
import torch

from gate_detection.models.gatenet import DEVICE, IMG_SIZE, GateNet, SYN_ROOT, corners_to_mask
from gate_detection.models.quadgate import (
    CORNER_TYPES,
    detect_corners_debug,
    parse_corners_from_label,
)

# One distinct colour per corner type (outer = warm, inner = cool)
_CORNER_COLORS = {
    "outer_TL": "#FF4444",
    "outer_TR": "#FF8C00",
    "outer_BR": "#FFD700",
    "outer_BL": "#ADFF2F",
    "inner_TL": "#00FFCC",
    "inner_TR": "#00BFFF",
    "inner_BR": "#6666FF",
    "inner_BL": "#CC44FF",
}

_COLS_SIMPLE = [
    "Image",
    "GT mask",
    "Prediction",
    "Pred B&W",
    "QuAdGate",
]

_COLS_DEBUG = [
    "Image",
    "GT mask",
    "Pred mask\n(QAG input)",
    "QAG 1\nlines",
    "QAG 2\ncandidates",
    "QAG 3\nmatches",
    "QAG 4\ncorners",
]


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------
def _find_label(img_path: str) -> str | None:
    stem   = os.path.splitext(os.path.basename(img_path))[0]
    flight = os.path.basename(os.path.dirname(os.path.dirname(img_path)))

    candidate = os.path.join(os.path.dirname(os.path.dirname(img_path)),
                             f"label_{flight}", stem + ".txt")
    if os.path.exists(candidate):
        return candidate

    for d in ("train", "val", "train_aug", "val_aug"):
        candidate = os.path.join(SYN_ROOT, d, stem + ".txt")
        if os.path.exists(candidate):
            return candidate

    return None


def _load_item(img_path: str):
    lbl_path = _find_label(img_path)
    img      = cv2.cvtColor(cv2.imread(img_path), cv2.COLOR_BGR2RGB)
    img      = cv2.resize(img, (IMG_SIZE, IMG_SIZE))
    img_t    = torch.from_numpy(img).permute(2, 0, 1).float() / 255.0
    mask     = corners_to_mask(lbl_path, IMG_SIZE) if lbl_path else \
               np.zeros((IMG_SIZE, IMG_SIZE), np.uint8)
    mask_t   = torch.from_numpy(mask).unsqueeze(0).float()
    return img_t, mask_t, os.path.basename(img_path)


# ---------------------------------------------------------------------------
# QuAdGate step renderers
# ---------------------------------------------------------------------------
def _render_lines(ax, mask_rot, lines_ext):
    """Step 1: derotated mask + extended LSD lines."""
    ax.imshow(mask_rot, cmap="gray", vmin=0, vmax=255)
    if lines_ext is not None and len(lines_ext):
        for x1, y1, x2, y2 in lines_ext:
            ax.plot([x1, x2], [y1, y2], color="cyan", linewidth=0.8, alpha=0.85)
        ax.set_title(f"{len(lines_ext)} lines", fontsize=7, pad=2)
    else:
        ax.set_title("no lines", fontsize=7, color="red", pad=2)
    ax.axis("off")


def _render_candidates(ax, mask_rot, cand_pts):
    """Step 2: all intersection candidates."""
    ax.imshow(mask_rot, cmap="gray", vmin=0, vmax=255)
    if cand_pts is not None and len(cand_pts):
        ax.scatter(cand_pts[:, 0], cand_pts[:, 1],
                   s=6, c="lime", marker=".", linewidths=0, alpha=0.75)
        ax.set_title(f"{len(cand_pts)} candidates", fontsize=7, pad=2)
    else:
        ax.set_title("no candidates", fontsize=7, color="red", pad=2)
    ax.axis("off")


def _render_matches(ax, mask_rot, priors_rot, matches):
    """Step 3: descriptor-matched candidates with priors (×) linked to matches (○)."""
    ax.imshow(mask_rot, cmap="gray", vmin=0, vmax=255)
    if priors_rot is not None:
        for i, ctype in enumerate(CORNER_TYPES):
            color = _CORNER_COLORS[ctype]
            px, py = priors_rot[i]
            # prior position
            ax.plot(px, py, "x", color=color, markersize=5, markeredgewidth=1.2,
                    label=ctype)
            if ctype in matches:
                mx, my = matches[ctype]
                # matched candidate
                ax.plot(mx, my, "o", color=color, markersize=4, markeredgewidth=0)
                # line from prior to match
                ax.plot([px, mx], [py, my], "-", color=color, linewidth=0.7, alpha=0.6)
    n = len(matches)
    color = "white" if n >= 3 else "red"
    ax.set_title(f"{n}/8 matched", fontsize=7, color=color, pad=2)
    ax.axis("off")


def _render_corners(ax, img_np, result):
    """Step 4: final refined corners overlaid on original image."""
    ax.imshow(img_np)
    if result is not None:
        ax.scatter(result[:, 0], result[:, 1], s=25, c="red", zorder=5, linewidths=0)
    else:
        ax.set_title("failed", fontsize=7, color="red", pad=2)
    ax.axis("off")


def _draw_qag_row(axes, img_np, pred_np, priors):
    """
    Fill axes[0..4] (cols 2-6) with the five QuAdGate pipeline steps.
    axes: [ax_pred, ax_lines, ax_cands, ax_matches, ax_final]
    """
    ax_pred, ax_lines, ax_cands, ax_matches, ax_final = axes

    debug = detect_corners_debug(pred_np, priors)

    # Col 2: prediction mask (QAG input)
    ax_pred.imshow(pred_np, cmap="gray", vmin=0, vmax=1)
    ax_pred.axis("off")

    mask_rot = debug["mask_rot"]

    _render_lines(ax_lines, mask_rot, debug["lines_ext"])
    _render_candidates(ax_cands, mask_rot, debug["cand_pts"])
    _render_matches(ax_matches, mask_rot, debug["priors_rot"], debug["matches"])
    _render_corners(ax_final, img_np, debug["result"])


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------
def run(checkpoint: str, n: int, out: str,
        images: list[str] = None, qag_debug: bool = False):
    model = GateNet(f=4).to(DEVICE)
    ckpt  = torch.load(checkpoint, map_location=DEVICE, weights_only=True)
    model.load_state_dict(ckpt["model"])
    model.eval()

    if images:
        paths = images
    else:
        all_imgs = glob.glob(
            os.path.join(SYN_ROOT, "camera_*", "*.jpg"),
            recursive=True,
        )
        paths = random.sample(all_imgs, min(n, len(all_imgs)))

    items  = [_load_item(p) for p in paths]
    n      = len(items)
    cols   = _COLS_DEBUG if qag_debug else _COLS_SIMPLE
    n_cols = len(cols)

    fig, axes = plt.subplots(n, n_cols, figsize=(n_cols * 4, n * 4))
    if n == 1:
        axes = axes[None, :]

    fig.suptitle(
        f"{os.path.basename(checkpoint)}  |  epoch {ckpt['epoch']}  |"
        f"  val_loss={ckpt['val_loss']:.4f}",
        fontsize=12,
    )
    for title, ax in zip(cols, axes[0]):
        ax.set_title(title, fontsize=9)

    for row, ((img_t, mask_t, fname), path) in enumerate(zip(items, paths)):
        with torch.no_grad():
            pred = model(img_t.unsqueeze(0).to(DEVICE))[-1]

        img_np  = img_t.permute(1, 2, 0).numpy()
        mask_np = mask_t.squeeze().numpy()
        pred_np = pred.squeeze().cpu().numpy()

        lbl_path   = _find_label(path)
        all_priors = parse_corners_from_label(lbl_path, IMG_SIZE)

        # Col 0: original image
        axes[row, 0].imshow(img_np)
        axes[row, 0].set_ylabel(fname, fontsize=6, rotation=0, labelpad=60, va="center")
        axes[row, 0].axis("off")

        # Col 1: GT mask overlay
        axes[row, 1].imshow(img_np)
        axes[row, 1].imshow(mask_np, cmap="Greens", alpha=0.6, vmin=0, vmax=1)
        axes[row, 1].axis("off")

        if qag_debug:
            # Cols 2-6: full QuAdGate pipeline steps
            if all_priors:
                _draw_qag_row(axes[row, 2:], img_np, pred_np, all_priors[0])
            else:
                for ax in axes[row, 2:]:
                    ax.imshow(pred_np, cmap="gray", vmin=0, vmax=1)
                    ax.set_title("no label", fontsize=7, color="gray", pad=2)
                    ax.axis("off")
        else:
            # Col 2: prediction overlay
            axes[row, 2].imshow(img_np)
            axes[row, 2].imshow(pred_np, cmap="Reds", alpha=0.6, vmin=0, vmax=1)
            axes[row, 2].axis("off")

            # Col 3: pred B&W
            axes[row, 3].imshow(pred_np, cmap="gray", vmin=0, vmax=1)
            axes[row, 3].axis("off")

            # Col 4: LSD lines + corner points (no connecting polygon)
            ax_qag = axes[row, 4]
            ax_qag.imshow(img_np)
            ax_qag.imshow(pred_np, cmap="gray", alpha=0.4, vmin=0, vmax=1)
            any_ok = False
            for priors in all_priors:
                debug = detect_corners_debug(pred_np, priors)
                if debug["lines_ext"] is not None:
                    for x1, y1, x2, y2 in debug["lines_ext"]:
                        ax_qag.plot([x1, x2], [y1, y2], color="cyan",
                                    linewidth=0.8, alpha=0.7)
                if debug["result"] is not None:
                    any_ok = True
                    pts = debug["result"]
                    ax_qag.scatter(pts[:, 0], pts[:, 1],
                                   s=25, c="red", zorder=5, linewidths=0)
            if not all_priors:
                ax_qag.set_title("no label", fontsize=8, color="gray")
            elif not any_ok:
                ax_qag.set_title("failed", fontsize=8, color="red")
            ax_qag.axis("off")

    plt.tight_layout()
    os.makedirs(os.path.dirname(out) or ".", exist_ok=True)
    plt.savefig(out, dpi=100, bbox_inches="tight")
    print(f"Saved -> {out}")


if __name__ == "__main__":
    p = argparse.ArgumentParser()
    p.add_argument("checkpoint", type=str)
    p.add_argument("--images",    type=str, nargs="+", default=None,
                   help="specific image paths; if omitted samples randomly from data/")
    p.add_argument("--n",         type=int, default=8,
                   help="number of random samples (ignored if --images is set)")
    p.add_argument("--out",       type=str, default=None,
                   help="output path (default: inference.png or inference_debug.png)")
    p.add_argument("--qag-debug", action="store_true",
                   help="show all QuAdGate intermediate steps (lines, candidates, matches)")
    args = p.parse_args()

    if args.out is None:
        args.out = "inference_debug.png" if args.qag_debug else "inference.png"

    run(args.checkpoint, args.n, args.out, args.images, args.qag_debug)
