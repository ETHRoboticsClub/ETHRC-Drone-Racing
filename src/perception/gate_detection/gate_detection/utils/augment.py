"""
Synthetic data augmentation pipeline (following the paper description).

For each curated sample in synthetic_labels/{train,val}/:
  1. Extract each gate region using its outer corners as a mask
  2. Apply random: perspective warp, scale, rotation
  3. Composite onto a random background frame from the dataset
  4. Apply HSV augmentation to gate and background independently
  5. Add Gaussian noise to the final composite
  6. Update label corner coordinates through the transforms

Output goes to synthetic_labels/train_aug/ and synthetic_labels/val_aug/.

Usage:
    python augment.py                     # 5x multiplier per sample
    python augment.py --mult 10 --split train
"""

import argparse
import glob
import os
import random

import cv2
import numpy as np

_GATE_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))

OUT_ROOT  = os.path.join(_GATE_ROOT, "data", "synthetic")
DATA_ROOT = os.path.join(_GATE_ROOT, "data")
BG_ROOT   = os.path.join(_GATE_ROOT, "data", "raw", "backgrounds")
IMG_W, IMG_H = 640, 480


# ---------------------------------------------------------------------------
# Label parsing
# ---------------------------------------------------------------------------
def parse_label(lbl_path: str):
    """Returns list of gate dicts with normalised inner+outer corners."""
    gates = []
    with open(lbl_path) as f:
        for line in f:
            vals = list(map(float, line.split()))
            if len(vals) < 29:
                continue
            inner, outer = [], []
            inner_vis, outer_vis = [], []
            for k in range(4):
                b = 5 + k * 3
                inner.append([vals[b], vals[b+1]])
                inner_vis.append(int(vals[b+2]))
            for k in range(4):
                b = 17 + k * 3
                outer.append([vals[b], vals[b+1]])
                outer_vis.append(int(vals[b+2]))
            gates.append({
                "cls": int(vals[0]),
                "cx": vals[1], "cy": vals[2], "w": vals[3], "h": vals[4],
                "inner": np.array(inner, np.float32),
                "inner_vis": inner_vis,
                "outer": np.array(outer, np.float32),
                "outer_vis": outer_vis,
            })
    return gates


def gates_to_lines(gates: list, img_w: int, img_h: int) -> list[str]:
    lines = []
    for g in gates:
        # Recompute bbox from outer corners (those that are in bounds)
        all_pts = []
        for pt, v in zip(g["outer"], g["outer_vis"]):
            if v > 0:
                all_pts.append(pt)
        for pt, v in zip(g["inner"], g["inner_vis"]):
            if v > 0:
                all_pts.append(pt)
        if not all_pts:
            continue
        all_pts = np.array(all_pts)
        cx = float(all_pts[:, 0].mean())
        cy = float(all_pts[:, 1].mean())
        w  = float(all_pts[:, 0].max() - all_pts[:, 0].min())
        h  = float(all_pts[:, 1].max() - all_pts[:, 1].min())

        line = f"{g['cls']} {cx} {cy} {w} {h}"
        for pt, v in zip(g["inner"], g["inner_vis"]):
            line += f" {pt[0]:.8f} {pt[1]:.8f} {v}"
        for pt, v in zip(g["outer"], g["outer_vis"]):
            line += f" {pt[0]:.8f} {pt[1]:.8f} {v}"
        lines.append(line)
    return lines


# ---------------------------------------------------------------------------
# Background pool
# ---------------------------------------------------------------------------
_bg_pool: list[str] = []

def get_bg_pool() -> list[str]:
    global _bg_pool
    if not _bg_pool:
        _bg_pool = glob.glob(os.path.join(BG_ROOT, "*.jpg"))
        if not _bg_pool:
            raise FileNotFoundError(
                f"No backgrounds found in {BG_ROOT}/. "
                "Run: python download_backgrounds.py"
            )
        random.shuffle(_bg_pool)
    return _bg_pool


def random_background() -> np.ndarray:
    pool = get_bg_pool()
    path = random.choice(pool)
    img  = cv2.imread(path)
    if img is not None:
        return cv2.resize(img, (IMG_W, IMG_H))
    return np.zeros((IMG_H, IMG_W, 3), np.uint8)


# ---------------------------------------------------------------------------
# HSV augmentation
# ---------------------------------------------------------------------------
def hsv_jitter(img: np.ndarray,
               h_range=10, s_range=0.2, v_range=0.2) -> np.ndarray:
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV).astype(np.float32)
    hsv[:, :, 0] = (hsv[:, :, 0] + random.uniform(-h_range, h_range)) % 180
    hsv[:, :, 1] = np.clip(hsv[:, :, 1] * random.uniform(1-s_range, 1+s_range), 0, 255)
    hsv[:, :, 2] = np.clip(hsv[:, :, 2] * random.uniform(1-v_range, 1+v_range), 0, 255)
    return cv2.cvtColor(hsv.astype(np.uint8), cv2.COLOR_HSV2BGR)


# ---------------------------------------------------------------------------
# Gate extraction + compositing
# ---------------------------------------------------------------------------
def gate_mask_from_outer(outer_px: np.ndarray, H: int, W: int) -> np.ndarray:
    mask = np.zeros((H, W), np.uint8)
    if len(outer_px) >= 3:
        cv2.fillPoly(mask, [outer_px.astype(np.int32)], 255)
    return mask


def random_perspective_warp(src_pts: np.ndarray, strength: float = 0.15):
    """Generate a random perspective transform matrix for the src_pts bounding box."""
    x0, y0 = src_pts.min(axis=0)
    x1, y1 = src_pts.max(axis=0)
    w = x1 - x0 or 1
    h = y1 - y0 or 1

    def jitter(val, frac):
        return val + random.uniform(-frac, frac) * max(w, h)

    src = np.float32([[x0, y0], [x1, y0], [x1, y1], [x0, y1]])
    dst = np.float32([
        [jitter(x0, strength), jitter(y0, strength)],
        [jitter(x1, strength), jitter(y0, strength)],
        [jitter(x1, strength), jitter(y1, strength)],
        [jitter(x0, strength), jitter(y1, strength)],
    ])
    M = cv2.getPerspectiveTransform(src, dst)
    return M


def transform_points(pts: np.ndarray, M: np.ndarray) -> np.ndarray:
    """Apply a 3x3 perspective matrix to (N,2) points."""
    ones = np.ones((len(pts), 1), dtype=np.float32)
    ph   = np.hstack([pts, ones])  # (N,3)
    t    = (M @ ph.T).T            # (N,3)
    return (t[:, :2] / t[:, 2:3])


# ---------------------------------------------------------------------------
# Single augmentation
# ---------------------------------------------------------------------------
def augment_sample(img_path: str, lbl_path: str,
                   out_img: str, out_lbl: str):
    stem  = os.path.splitext(os.path.basename(img_path))[0]
    img   = cv2.imread(img_path)
    if img is None:
        return False
    H, W  = img.shape[:2]
    gates = parse_label(lbl_path)
    if not gates:
        return False

    # --- Background (HSV jittered separately) ---
    bg = random_background()
    bg = hsv_jitter(bg)

    # --- Scale + rotate transform on full image ---
    scale = random.uniform(0.85, 1.15)
    angle = random.uniform(-15, 15)
    tx    = random.uniform(-0.08, 0.08) * W
    ty    = random.uniform(-0.08, 0.08) * H
    M_aff = cv2.getRotationMatrix2D((W/2, H/2), angle, scale)
    M_aff[0, 2] += tx
    M_aff[1, 2] += ty
    M_aff_3x3 = np.vstack([M_aff, [0, 0, 1]])

    warped_gate = cv2.warpAffine(img, M_aff, (W, H),
                                 flags=cv2.INTER_LINEAR,
                                 borderMode=cv2.BORDER_REFLECT)

    # --- Random perspective warp ---
    # Build corners of the full image as src
    src = np.float32([[0,0],[W,0],[W,H],[0,H]])
    strength = random.uniform(0.0, 0.12)
    dst = src + np.random.uniform(-strength * min(W, H),
                                   strength * min(W, H),
                                   src.shape).astype(np.float32)
    M_persp = cv2.getPerspectiveTransform(src, dst)
    M_full  = M_persp @ M_aff_3x3   # combined transform

    warped_gate = cv2.warpPerspective(img, M_full, (W, H),
                                      flags=cv2.INTER_LINEAR,
                                      borderMode=cv2.BORDER_REFLECT)

    # --- HSV jitter on gate independently ---
    warped_gate = hsv_jitter(warped_gate)

    # --- Build combined mask of all gate outer polygons ---
    combined_mask = np.zeros((H, W), np.uint8)
    for g in gates:
        vis_outer = [(pt, v) for pt, v in zip(g["outer"], g["outer_vis"]) if v > 0]
        if len(vis_outer) < 3:
            vis_outer = [(pt, v) for pt, v in zip(g["inner"], g["inner_vis"]) if v > 0]
        if len(vis_outer) < 3:
            continue
        pts_norm  = np.array([p for p, _ in vis_outer], np.float32)
        pts_px    = pts_norm * [W, H]
        pts_warped = transform_points(pts_px, M_full).astype(np.int32)
        cv2.fillPoly(combined_mask, [pts_warped], 255)

    # Dilate mask slightly to avoid fringe
    kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (7, 7))
    combined_mask = cv2.dilate(combined_mask, kernel)

    # --- Composite: gate over background ---
    mask3 = combined_mask[:, :, None] / 255.0
    composite = (warped_gate * mask3 + bg * (1 - mask3)).astype(np.uint8)

    # --- Gaussian noise on full composite ---
    sigma = random.uniform(0, 12)
    noise = np.random.normal(0, sigma, composite.shape).astype(np.float32)
    composite = np.clip(composite.astype(np.float32) + noise, 0, 255).astype(np.uint8)

    # --- Transform label coordinates ---
    new_gates = []
    for g in gates:
        def transform_corners(pts_norm, vis_list):
            new_pts, new_vis = [], []
            for pt, v in zip(pts_norm, vis_list):
                if v > 0:
                    px = np.array([[pt[0] * W, pt[1] * H]], np.float32)
                    tp = transform_points(px, M_full)[0]
                    nx, ny = tp[0] / W, tp[1] / H
                    in_bounds = 0.0 <= nx <= 1.0 and 0.0 <= ny <= 1.0
                    new_pts.append([nx, ny])
                    new_vis.append(2 if in_bounds else 0)
                else:
                    new_pts.append([0.0, 0.0])
                    new_vis.append(0)
            return np.array(new_pts, np.float32), new_vis

        new_inner, new_inner_vis = transform_corners(g["inner"], g["inner_vis"])
        new_outer, new_outer_vis = transform_corners(g["outer"], g["outer_vis"])

        # Skip gate if no corners remain in bounds
        if not any(v > 0 for v in new_inner_vis):
            continue

        new_gates.append({**g,
                          "inner": new_inner, "inner_vis": new_inner_vis,
                          "outer": new_outer, "outer_vis": new_outer_vis})

    if not new_gates:
        return False

    cv2.imwrite(out_img, composite)
    lines = gates_to_lines(new_gates, W, H)
    with open(out_lbl, "w") as f:
        f.write("\n".join(lines) + "\n")
    return True


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------
def main(splits: list[str], mult: int):
    for split in splits:
        in_dir  = os.path.join(OUT_ROOT, split)
        out_dir = os.path.join(OUT_ROOT, f"{split}_aug")
        os.makedirs(out_dir, exist_ok=True)

        stems = sorted({os.path.splitext(f)[0]
                        for f in os.listdir(in_dir) if f.endswith(".txt")})
        print(f"{split}: {len(stems)} curated samples × {mult} → target {len(stems)*mult}")

        generated = 0
        for stem in stems:
            img_p = os.path.join(in_dir, stem + ".jpg")
            lbl_p = os.path.join(in_dir, stem + ".txt")
            for j in range(mult):
                out_img = os.path.join(out_dir, f"{stem}_aug{j:02d}.jpg")
                out_lbl = os.path.join(out_dir, f"{stem}_aug{j:02d}.txt")
                if os.path.exists(out_img):
                    generated += 1
                    continue
                ok = augment_sample(img_p, lbl_p, out_img, out_lbl)
                if ok:
                    generated += 1

        print(f"  → {generated} augmented samples in {out_dir}/")


if __name__ == "__main__":
    p = argparse.ArgumentParser()
    p.add_argument("--split", nargs="+", default=["train", "val"])
    p.add_argument("--mult",  type=int, default=5,
                   help="augmented copies per curated sample")
    args = p.parse_args()
    main(args.split, args.mult)
