#!/usr/bin/env -S uv run --quiet
# /// script
# requires-python = ">=3.10"
# dependencies = [
#   "reportlab>=4.0",
# ]
# ///
"""
Generate a printable checkerboard PDF for camera intrinsics calibration.

Produces a 9x6 *inner-corner* checkerboard (10x7 squares), 35 mm nominal
square size, centred on A3 landscape. Output goes to
``camera_calibration/checkerboard_9x6_35mm_a3.pdf``.

After printing, MEASURE one square with a ruler — printers commonly scale
by 1-3 percent. Pass the measured size to ``calibrate_imx219.py`` via
``--square-size-mm``.

Usage:
    ./generate_checkerboard.py
    ./generate_checkerboard.py --inner-cols 9 --inner-rows 6 --square-mm 35
"""
from __future__ import annotations

import argparse
import sys
from pathlib import Path

from reportlab.lib.pagesizes import A3, landscape
from reportlab.lib.units import mm
from reportlab.pdfgen import canvas


def build_pdf(out_path: Path, inner_cols: int, inner_rows: int, square_mm: float) -> None:
    cols = inner_cols + 1  # squares per row
    rows = inner_rows + 1  # squares per column

    page_w, page_h = landscape(A3)  # mm-equivalent in points
    board_w = cols * square_mm * mm
    board_h = rows * square_mm * mm

    if board_w > page_w or board_h > page_h:
        sys.exit(
            f"[ERROR] board ({board_w/mm:.0f}x{board_h/mm:.0f} mm) larger than "
            f"A3 landscape ({page_w/mm:.0f}x{page_h/mm:.0f} mm). "
            f"Reduce --square-mm or pattern."
        )

    margin_x = (page_w - board_w) / 2
    margin_y = (page_h - board_h) / 2

    c = canvas.Canvas(str(out_path), pagesize=landscape(A3))
    c.setTitle(f"Checkerboard {inner_cols}x{inner_rows} inner @ {square_mm} mm")

    # Squares: standard convention is top-left = black. We draw filled black
    # squares only, on white background. (row + col) even => black.
    for r in range(rows):
        for col in range(cols):
            if (r + col) % 2 == 0:
                x = margin_x + col * square_mm * mm
                # PDF origin = bottom-left; flip row index
                y = margin_y + (rows - 1 - r) * square_mm * mm
                c.rect(x, y, square_mm * mm, square_mm * mm, stroke=0, fill=1)

    # Footer caption (outside the board, in the bottom margin)
    c.setFont("Helvetica", 9)
    caption = (
        f"Checkerboard  inner corners {inner_cols}x{inner_rows}  "
        f"(squares {cols}x{rows})  square = {square_mm:.1f} mm nominal  "
        f"page A3 landscape  -  MEASURE PRINTED SQUARE BEFORE CALIBRATION"
    )
    c.drawCentredString(page_w / 2, margin_y / 2, caption)
    c.showPage()
    c.save()


def main() -> int:
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--inner-cols", type=int, default=9)
    p.add_argument("--inner-rows", type=int, default=6)
    p.add_argument("--square-mm", type=float, default=35.0)
    p.add_argument(
        "--out",
        type=Path,
        default=Path(__file__).resolve().parents[2] / "camera_calibration" / "checkerboard_9x6_35mm_a3.pdf",
    )
    args = p.parse_args()
    args.out.parent.mkdir(parents=True, exist_ok=True)
    build_pdf(args.out, args.inner_cols, args.inner_rows, args.square_mm)
    print(f"[OK] wrote {args.out}")
    print(
        f"[INFO] pattern: {args.inner_cols}x{args.inner_rows} inner corners, "
        f"{args.square_mm} mm nominal squares"
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
