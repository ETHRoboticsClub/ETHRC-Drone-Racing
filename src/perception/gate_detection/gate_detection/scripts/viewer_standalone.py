"""
Interactive label viewer with keep/discard.

Controls:
  SPACE or K  → keep, next
  D           → discard (deletes .jpg + .txt from whichever split it's in), next
  B           → back (view previous, cannot un-discard)
  Q / ESC     → quit

Usage:
    python viewer.py
    python viewer.py --shuffle
"""

import argparse
import os
import random

_GATE_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))

import cv2
import numpy as np

OUT_ROOT    = os.path.join(_GATE_ROOT, "data", "synthetic")
REVIEWED_LOG = os.path.join(OUT_ROOT, ".reviewed")


def load_reviewed() -> set[str]:
    if not os.path.exists(REVIEWED_LOG):
        return set()
    with open(REVIEWED_LOG) as f:
        return {l.strip() for l in f if l.strip()}


def mark_reviewed(stem: str):
    with open(REVIEWED_LOG, "a") as f:
        f.write(stem + "\n")


def load_samples(shuffle: bool, all_samples: bool) -> list[tuple[str, str]]:
    """Return list of (stem, split_dir). If not all_samples, skip already reviewed."""
    reviewed = set() if all_samples else load_reviewed()
    entries  = []
    for split in ("train", "val"):
        split_dir = os.path.join(OUT_ROOT, split)
        if not os.path.isdir(split_dir):
            continue
        for f in sorted(os.listdir(split_dir)):
            if f.endswith(".txt"):
                stem = os.path.splitext(f)[0]
                if stem not in reviewed:
                    entries.append((stem, split_dir))
    if shuffle:
        random.shuffle(entries)
    return entries


def parse_label(lbl_path: str):
    gates = []
    with open(lbl_path) as f:
        for line in f:
            vals = list(map(float, line.split()))
            if len(vals) < 29:
                continue
            inner, outer = [], []
            for k in range(4):
                b = 5 + k * 3
                if vals[b+2] > 0:
                    inner.append([vals[b], vals[b+1]])
            for k in range(4):
                b = 17 + k * 3
                if vals[b+2] > 0:
                    outer.append([vals[b], vals[b+1]])
            gates.append((np.array(inner, dtype=np.float32),
                          np.array(outer, dtype=np.float32)))
    return gates


def draw_frame(img_bgr: np.ndarray, gates: list,
               idx: int, total: int, stem: str) -> np.ndarray:
    H, W = img_bgr.shape[:2]
    vis  = img_bgr.copy()

    for inner_n, outer_n in gates:
        if len(inner_n) >= 2:
            pts = (inner_n * [W, H]).astype(np.int32)
            cv2.polylines(vis, [pts], isClosed=len(pts)==4, color=(0, 255, 0), thickness=2)
            for p in pts:
                cv2.circle(vis, tuple(p), 6, (0, 255, 0), -1)
                cv2.circle(vis, tuple(p), 6, (0, 0, 0), 1)

        if len(outer_n) >= 2:
            pts = (outer_n * [W, H]).astype(np.int32)
            cv2.polylines(vis, [pts], isClosed=len(pts)==4, color=(0, 165, 255), thickness=2)
            for p in pts:
                cv2.circle(vis, tuple(p), 6, (0, 165, 255), -1)
                cv2.circle(vis, tuple(p), 6, (0, 0, 0), 1)

    # Top bar
    cv2.rectangle(vis, (0, 0), (W, 38), (30, 30, 30), -1)
    cv2.putText(vis, f"[{idx+1}/{total}]  {stem}",
                (8, 24), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 1, cv2.LINE_AA)

    # Bottom bar
    cv2.rectangle(vis, (0, H - 30), (W, H), (30, 30, 30), -1)
    cv2.putText(vis, "SPACE/K=keep   D=discard   B=back   Q=quit",
                (8, H - 9), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (180, 180, 180), 1, cv2.LINE_AA)

    return vis


def run(shuffle: bool, all_samples: bool):
    samples   = load_samples(shuffle, all_samples)
    discarded = 0

    if not samples:
        print("No new samples to review. Use --all to review everything.")
        return
    print(f"{len(samples)} {'total' if all_samples else 'new'} samples. SPACE/K=keep  D=discard  B=back  Q=quit")

    cv2.namedWindow("viewer", cv2.WINDOW_NORMAL)
    cv2.resizeWindow("viewer", 900, 700)

    i = 0
    while i < len(samples):
        stem, split_dir = samples[i]
        img_p = os.path.join(split_dir, stem + ".jpg")
        lbl_p = os.path.join(split_dir, stem + ".txt")

        if not os.path.exists(img_p):
            samples.pop(i)
            continue

        img   = cv2.imread(img_p)
        gates = parse_label(lbl_p)
        split_name = os.path.basename(split_dir)
        frame = draw_frame(img, gates, i, len(samples), f"[{split_name}] {stem}")
        cv2.imshow("viewer", frame)

        key = cv2.waitKey(0) & 0xFF

        if key in (ord("q"), 27):
            break
        elif key == ord("b"):
            i = max(0, i - 1)
        elif key == ord("d"):
            os.remove(img_p)
            os.remove(lbl_p)
            samples.pop(i)
            discarded += 1
            mark_reviewed(stem)
            print(f"  Discarded: {stem}  ({discarded} total discarded)")
        else:                               # SPACE / K → keep
            mark_reviewed(stem)
            i += 1

    cv2.destroyAllWindows()
    print(f"\nSession done. Discarded {discarded} samples.")
    if discarded:
        print(f"Run 'python generate_labels.py' to top up the missing {discarded}.")


if __name__ == "__main__":
    p = argparse.ArgumentParser()
    p.add_argument("--shuffle", action="store_true")
    p.add_argument("--all",     action="store_true", help="review all, not just new")
    args = p.parse_args()
    run(args.shuffle, args.all)
