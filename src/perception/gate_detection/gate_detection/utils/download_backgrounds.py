"""
Download background images for synthetic data augmentation.
Uses the keremberke/indoor-scene-classification dataset from HuggingFace
and filters for relevant indoor environments.

Usage:
    pip install datasets pillow
    python download_backgrounds.py
    python download_backgrounds.py --n 200 --out backgrounds/
"""

import argparse
import os

_GATE_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))

OUT_DIR = os.path.join(_GATE_ROOT, "data", "raw", "backgrounds")

# Categories that resemble drone racing environments
RELEVANT = {
    "airport_inside", "corridor", "gym", "gymnasium",
    "inside_bus", "inside_subway", "lobby",
    "mall", "office", "restaurant", "shopping_mall",
    "subway", "warehouse", "elevator", "garage",
    "indoor_parking", "staircase", "children_room",
    "classroom", "computer_room", "conference_room",
    "dining_room", "exhibition_room", "game_room",
    "gym", "hall", "library", "living_room",
    "sports_indoor", "swimming_pool_indoor",
    "train_station", "waiting_room",
}


def download(n: int, out_dir: str):
    try:
        from datasets import load_dataset
    except ImportError:
        print("Install: uv pip install datasets pillow")
        return

    os.makedirs(out_dir, exist_ok=True)

    print("Loading dataset index (streaming)...")
    ds = load_dataset("Andron00e/Places365-custom",
                      split="train", streaming=True, trust_remote_code=True)

    saved = 0
    for sample in ds:
        if saved >= n:
            break

        img = sample.get("image") or sample.get("img")
        if img is None:
            continue

        # Convert PIL → save as JPEG
        path = os.path.join(out_dir, f"bg_{saved:04d}.jpg")
        img = img.convert("RGB").resize((640, 480))
        img.save(path, "JPEG", quality=90)
        saved += 1
        if saved % 20 == 0:
            print(f"  {saved}/{n}")

    print(f"\nSaved {saved} backgrounds to {out_dir}/")


if __name__ == "__main__":
    p = argparse.ArgumentParser()
    p.add_argument("--n",   type=int, default=300, help="number of images to download")
    p.add_argument("--out", type=str, default=OUT_DIR)
    args = p.parse_args()
    download(args.n, args.out)
