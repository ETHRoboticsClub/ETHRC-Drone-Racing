"""
QuAdGate: Sub-pixel gate corner detection from segmentation masks.

Extracts inner and outer gate corners by:
  1. Derotating the mask using estimated attitude angle
  2. Detecting line segments with LSD
  3. Extending lines by 5/3 and computing pairwise intersections → candidates
  4. Matching candidates to EKF/GT priors via descriptors + RANSAC
"""

import cv2
import numpy as np


CORNER_TYPES = [
    "outer_TL", "outer_TR", "outer_BR", "outer_BL",
    "inner_TL", "inner_TR", "inner_BR", "inner_BL",
]

PRIOR_DESCRIPTORS = {
    "outer_TL": np.array([0, 0, 1, 0]),
    "outer_TR": np.array([0, 0, 0, 1]),
    "outer_BR": np.array([1, 0, 0, 0]),
    "outer_BL": np.array([0, 1, 0, 0]),
    "inner_TL": np.array([1, 1, 0, 1]),
    "inner_TR": np.array([1, 1, 1, 0]),
    "inner_BR": np.array([0, 1, 1, 1]),
    "inner_BL": np.array([1, 0, 1, 1]),
}


def _derotate(mask: np.ndarray, angle_deg: float):
    """Rotate mask so world-up aligns with image-up. Returns (rotated_mask, M_2x3)."""
    h, w = mask.shape
    M = cv2.getRotationMatrix2D((w / 2, h / 2), angle_deg, 1.0)
    rotated = cv2.warpAffine(mask, M, (w, h), flags=cv2.INTER_NEAREST)
    return rotated, M


def _detect_lines(mask_u8: np.ndarray) -> np.ndarray:
    """Run LSD on binary mask; returns (N,4) array of [x1,y1,x2,y2] or empty."""
    lsd = cv2.createLineSegmentDetector(
        cv2.LSD_REFINE_STD,
        0.8,   # scale
        0.8,   # sigma_scale
        25.0,  # quant
        30.0,  # ang_th
    )
    lines, *_ = lsd.detect(mask_u8)
    if lines is None or len(lines) == 0:
        return np.empty((0, 4), dtype=np.float32)
    return lines.reshape(-1, 4).astype(np.float32)


def _extend_lines(lines: np.ndarray) -> np.ndarray:
    """Extend each segment symmetrically so total length grows by 5/3."""
    cx = (lines[:, 0] + lines[:, 2]) / 2
    cy = (lines[:, 1] + lines[:, 3]) / 2
    hx = (lines[:, 2] - lines[:, 0]) / 2
    hy = (lines[:, 3] - lines[:, 1]) / 2
    f = 5 / 3
    ext = np.stack([cx - f * hx, cy - f * hy, cx + f * hx, cy + f * hy], axis=1)
    return ext


def _clip_lines(lines: np.ndarray, w: int, h: int) -> np.ndarray:
    """Clip line segments to [0, w) × [0, h) using parametric clipping.
    Segments fully outside the image are dropped."""
    clipped = []
    for x1, y1, x2, y2 in lines:
        dx, dy = x2 - x1, y2 - y1
        t0, t1 = 0.0, 1.0
        for p, q in [(-dx, x1), (dx, (w - 1) - x1),
                     (-dy, y1), (dy, (h - 1) - y1)]:
            if p == 0:
                if q < 0:
                    t0 = 2.0  # mark as rejected
                    break
            else:
                t = q / p
                if p < 0:
                    t0 = max(t0, t)
                else:
                    t1 = min(t1, t)
        if t0 <= t1:
            clipped.append([x1 + t0 * dx, y1 + t0 * dy,
                             x1 + t1 * dx, y1 + t1 * dy])
    return np.array(clipped, dtype=np.float32) if clipped else np.empty((0, 4), dtype=np.float32)


def _intersect_lines(lines: np.ndarray, img_shape: tuple) -> list:
    """
    Compute all pairwise intersections of lines (treated as infinite).
    Returns list of (pt, d1, d2) where pt is (x,y) and d1/d2 are unit direction vectors.
    Only points inside the image are kept.
    """
    h, w = img_shape
    results = []
    n = len(lines)
    for i in range(n):
        x1, y1, x2, y2 = lines[i]
        for j in range(i + 1, n):
            x3, y3, x4, y4 = lines[j]
            denom = (x1 - x2) * (y3 - y4) - (y1 - y2) * (x3 - x4)
            if abs(denom) < 1e-8:
                continue  # parallel
            t = ((x1 - x3) * (y3 - y4) - (y1 - y3) * (x3 - x4)) / denom
            px = x1 + t * (x2 - x1)
            py = y1 + t * (y2 - y1)
            if not (0 <= px < w and 0 <= py < h):
                continue
            d1 = np.array([x2 - x1, y2 - y1], dtype=np.float64)
            d2 = np.array([x4 - x3, y4 - y3], dtype=np.float64)
            d1 /= np.linalg.norm(d1) + 1e-12
            d2 /= np.linalg.norm(d2) + 1e-12
            results.append((np.array([px, py]), d1, d2))
    return results


def _sample_descriptor(mask: np.ndarray, pt: np.ndarray, d1: np.ndarray, d2: np.ndarray) -> np.ndarray:
    """
    Extract d(p) = [vTL, vTR, vBR, vBL] at candidate corner pt.

    Classify d1/d2 into horizontal vs vertical direction, ensure h_dir points
    right (+x) and v_dir points down (+y image coords = world down after derotation).
    Sample 5 px diagonally along each quadrant.
    """
    h, w = mask.shape

    # Assign horizontal vs vertical based on which direction is more aligned to x-axis
    if abs(d1[0]) >= abs(d2[0]):
        h_dir, v_dir = d1.copy(), d2.copy()
    else:
        h_dir, v_dir = d2.copy(), d1.copy()

    # Canonicalise: h_dir → right (+x), v_dir → down (+y)
    if h_dir[0] < 0:
        h_dir = -h_dir
    if v_dir[1] < 0:
        v_dir = -v_dir

    step = 5.0

    def sample(offset):
        p = pt + offset
        xi, yi = int(round(p[0])), int(round(p[1]))
        if 0 <= xi < w and 0 <= yi < h:
            return 1.0 if mask[yi, xi] > 0 else 0.0
        return 0.0

    vTL = sample(-step * h_dir - step * v_dir)
    vTR = sample(+step * h_dir - step * v_dir)
    vBR = sample(+step * h_dir + step * v_dir)
    vBL = sample(-step * h_dir + step * v_dir)

    return np.array([vTL, vTR, vBR, vBL])


def assign_corner_types(corners_outer: np.ndarray, corners_inner: np.ndarray) -> np.ndarray:
    """
    Given (4,2) outer and (4,2) inner corners (any order), sort each group into
    [TL, TR, BR, BL] order and return (8,2) array matching CORNER_TYPES.

    Sorting: TL = min(x+y), TR = max(x-y), BR = max(x+y), BL = min(x-y).
    """
    def sort_corners(pts):
        s = pts[:, 0] + pts[:, 1]   # x+y
        d = pts[:, 0] - pts[:, 1]   # x-y
        TL = pts[np.argmin(s)]
        TR = pts[np.argmax(d)]
        BR = pts[np.argmax(s)]
        BL = pts[np.argmin(d)]
        return np.stack([TL, TR, BR, BL])

    outer_sorted = sort_corners(corners_outer)
    inner_sorted = sort_corners(corners_inner)
    return np.vstack([outer_sorted, inner_sorted])  # (8,2)

# Save every step for debugging
def detect_corners_debug(
    mask: np.ndarray,
    priors: np.ndarray,
    attitude_angle: float = 0.0,
    max_dist: float = 100.0,
    ransac_threshold: float = 5.0,
    max_translation: float = 150.0,
) -> dict:
    """
    QuAdGate corner detection with full intermediate outputs for visualization.

    Returns a dict with keys:
        mask_rot    : derotated binary mask (H, W, uint8 0/255)
        M_rot       : 2x3 rotation matrix used for derotation
        priors_rot  : (8, 2) priors in derotated frame
        lines       : (N, 4) raw LSD segments [x1,y1,x2,y2]
        lines_ext   : (N, 4) extended segments
        cand_pts    : (M, 2) intersection candidate positions
        cand_descs  : (M, 4) binary descriptors for each candidate
        matches     : dict {corner_type: (x, y)} for descriptor-matched candidates
        M_affine    : 2x3 RANSAC affine matrix (or None)
        result      : (8, 2) refined corners in original frame (or None on failure)
    """
    out = dict(mask_rot=None, M_rot=None, priors_rot=None,
               lines=None, lines_ext=None, cand_pts=None, cand_descs=None,
               matches={}, M_affine=None, result=None)

    mask_bin = (mask > 0.5).astype(np.uint8) * 255

    # 1. Derotate
    mask_rot, M_rot = _derotate(mask_bin, attitude_angle)
    out["mask_rot"] = mask_rot
    out["M_rot"]    = M_rot

    priors_h   = np.hstack([priors.astype(np.float64), np.ones((len(priors), 1))])
    priors_rot = (M_rot @ priors_h.T).T  # (8, 2)
    out["priors_rot"] = priors_rot

    h, w = mask_rot.shape

    # 2. Detect + extend lines
    lines = _detect_lines(mask_rot)
    out["lines"] = lines
    if len(lines) < 2:
        return out
    lines_ext = _clip_lines(_extend_lines(lines), w, h)
    out["lines_ext"] = lines_ext

    # 3. Compute intersections → candidates
    candidates = _intersect_lines(lines_ext, (h, w))
    if not candidates:
        return out

    cand_pts   = np.array([c[0] for c in candidates])
    cand_descs = np.array([
        _sample_descriptor(mask_rot, c[0], c[1], c[2]) for c in candidates
    ])
    out["cand_pts"]   = cand_pts
    out["cand_descs"] = cand_descs

    # 4. Match each prior to best descriptor-compatible candidate
    matched_priors = []
    matched_cands  = []
    matches        = {}

    for prior_idx, ctype in enumerate(CORNER_TYPES):
        prior_pt   = priors_rot[prior_idx]
        prior_desc = PRIOR_DESCRIPTORS[ctype]

        desc_match = np.all(cand_descs == prior_desc, axis=1)
        dists      = np.linalg.norm(cand_pts - prior_pt, axis=1)
        valid      = np.where(desc_match & (dists <= max_dist))[0]

        if len(valid) == 0:
            continue

        best = valid[np.argmin(dists[valid])]
        matched_priors.append(prior_pt)
        matched_cands.append(cand_pts[best])
        matches[ctype] = cand_pts[best]

    out["matches"] = matches

    if len(matched_priors) < 3:
        return out

    src = np.array(matched_priors, dtype=np.float32)
    dst = np.array(matched_cands,  dtype=np.float32)

    # 5. RANSAC 4-DOF affine (translation + rotation + uniform scale)
    M_affine, _ = cv2.estimateAffinePartial2D(
        src, dst,
        method=cv2.RANSAC,
        ransacReprojThreshold=ransac_threshold,
    )
    out["M_affine"] = M_affine

    if M_affine is None:
        return out
    if abs(M_affine[0, 2]) > max_translation or abs(M_affine[1, 2]) > max_translation:
        return out

    # 6. Apply affine to all priors → refined positions in derotated frame
    priors_rot_h = np.hstack([priors_rot, np.ones((len(priors_rot), 1))])
    refined_rot  = (M_affine @ priors_rot_h.T).T

    # 7. Un-derotate back to original frame
    M_rot_inv  = cv2.invertAffineTransform(M_rot)
    refined_h  = np.hstack([refined_rot, np.ones((len(refined_rot), 1))])
    refined    = (M_rot_inv @ refined_h.T).T

    # Reject if any corner falls outside the image
    if np.any(refined < 0) or np.any(refined[:, 0] >= w) or np.any(refined[:, 1] >= h):
        return out

    out["result"] = refined.astype(np.float32)
    return out


def detect_corners(
    mask: np.ndarray,
    priors: np.ndarray,
    attitude_angle: float = 0.0,
    max_dist: float = 100.0,
    ransac_threshold: float = 5.0,
    max_translation: float = 150.0,
) -> np.ndarray | None:
    """QuAdGate corner detection. Returns (8, 2) refined corners or None on failure."""
    return detect_corners_debug(
        mask, priors, attitude_angle, max_dist, ransac_threshold, max_translation,
    )["result"]


def parse_corners_from_label(
    label_path: str,
    img_size: int,
    outer_scale: float = 1.4,
) -> list[np.ndarray]:
    """
    Parse all gates from a label file.

    Supports both formats:
      - 17-col (original): inner corners only — outer corners are synthesised by
        scaling radially from the gate centroid (same method as generate_labels.py).
      - 29-col (synthetic): inner + outer corners read directly.

    Returns a list of (8, 2) pixel-coordinate arrays, one per gate, ordered as
    CORNER_TYPES: [outer_TL, outer_TR, outer_BR, outer_BL,
                   inner_TL, inner_TR, inner_BR, inner_BL].
    Returns an empty list if the file is missing or unreadable.
    """
    import os as _os
    if not label_path or not _os.path.exists(label_path):
        return []

    gates = []
    with open(label_path) as f:
        for line in f:
            vals = list(map(float, line.split()))
            if len(vals) < 17:
                continue

            def read_corners(start, n):
                pts = []
                for k in range(n):
                    b = start + k * 3
                    x, y, vis = vals[b], vals[b + 1], vals[b + 2]
                    if vis > 0:
                        pts.append([x * img_size, y * img_size])
                return pts

            inner_list = read_corners(5, 4)
            if len(inner_list) < 3:
                continue
            inner = np.array(inner_list, dtype=np.float32)

            if len(vals) >= 29:
                outer_list = read_corners(17, 4)
                if len(outer_list) < 3:
                    continue
                outer = np.array(outer_list, dtype=np.float32)
            else:
                # Synthesise outer corners by radial scaling from centroid
                centroid = inner.mean(axis=0)
                outer = centroid + outer_scale * (inner - centroid)

            gates.append(assign_corner_types(outer, inner))

    return gates
