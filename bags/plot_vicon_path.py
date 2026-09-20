#!/usr/bin/env -S uv run --quiet
# /// script
# requires-python = ">=3.10"
# dependencies = [
#   "numpy>=1.24",
#   "rosbags>=0.10",
#   "matplotlib>=3.7",
# ]
# ///

# TODO
#################################################
# This file exists for backwards compatibility. #
# Use tools/plot_vicon_path.py instead.         #
#################################################

"""
Plot Vicon XY trajectory and body +X / +Y axes from a ROS 2 bag.

Reads /vicon/boreas/boreas (geometry_msgs/PoseStamped) directly from
the bag — no need to run ``ros2 bag play`` in another terminal.

Usage
-----
    cd bags
    ./plot_vicon_path.py --save vicon_path.png
    ./plot_vicon_path.py --bag 2026-05-17-zephyrus-multi-gate2 --save plot.png
    ./plot_vicon_path.py --t-start 20 --t-end 40 --save vicon_path.png
"""
from __future__ import annotations

import argparse
import sys
from pathlib import Path

import matplotlib.pyplot as plt
from matplotlib.collections import LineCollection
from matplotlib.colors import LinearSegmentedColormap
import numpy as np
from rosbags.highlevel import AnyReader
from rosbags.typesys import Stores, get_typestore

BAGS_DIR = Path(__file__).resolve().parent
DEFAULT_BAG_NAME = "2026-05-17-zephyrus-multi-gate2"
DEFAULT_BAG = BAGS_DIR / DEFAULT_BAG_NAME
DEFAULT_TOPIC = "/vicon/boreas/boreas"

RED_BLUE_CMAP = LinearSegmentedColormap.from_list(
    "red_blue", ["#ff0000", "#0000ff"]
)
WHITE_BLACK_CMAP = RED_BLUE_CMAP
WHITE_RED_CMAP = RED_BLUE_CMAP
BLACK_RED_CMAP = RED_BLUE_CMAP


def _normalize_xy(v: np.ndarray) -> np.ndarray:
    norm = np.linalg.norm(v)
    return v / norm if norm > 0.0 else v


def body_x_axis_world(qx: float, qy: float, qz: float, qw: float) -> np.ndarray:
    """Unit vector of body +X axis in the world frame (ROS quat x,y,z,w)."""
    return _normalize_xy(
        np.array(
            [
                1.0 - 2.0 * (qy * qy + qz * qz),
                2.0 * (qx * qy + qw * qz),
                2.0 * (qx * qz - qw * qy),
            ],
            dtype=np.float64,
        )
    )


def body_y_axis_world(qx: float, qy: float, qz: float, qw: float) -> np.ndarray:
    """Unit vector of body +Y axis in the world frame (ROS quat x,y,z,w)."""
    return _normalize_xy(
        np.array(
            [
                2.0 * (qx * qy - qw * qz),
                1.0 - 2.0 * (qx * qx + qz * qz),
                2.0 * (qy * qz + qw * qx),
            ],
            dtype=np.float64,
        )
    )


def load_vicon_poses(
    bag_dir: Path,
    topic: str = DEFAULT_TOPIC,
) -> tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray, int]:
    """Return times (s), positions Nx2, +X/+Y axes Nx2, and skipped count."""
    typestore = get_typestore(Stores.ROS2_HUMBLE)
    xs: list[float] = []
    ys: list[float] = []
    x_axes: list[np.ndarray] = []
    y_axes: list[np.ndarray] = []
    times: list[float] = []

    with AnyReader([bag_dir], default_typestore=typestore) as reader:
        connections = [c for c in reader.connections if c.topic == topic]
        if not connections:
            available = sorted({c.topic for c in reader.connections})
            sys.exit(
                f"[ERROR] topic {topic!r} not in bag.\n"
                f"Available topics: {available}"
            )

        t0_ns: int | None = None
        skipped_invalid = 0
        for _conn, timestamp, raw in reader.messages(connections=connections):
            msg = reader.deserialize(raw, _conn.msgtype)
            p = msg.pose.position
            x, y = float(p.x), float(p.y)
            if x == 0.0 and y == 0.0:
                skipped_invalid += 1
                continue

            if t0_ns is None:
                t0_ns = timestamp
            times.append((timestamp - t0_ns) * 1e-9)

            o = msg.pose.orientation
            xs.append(x)
            ys.append(y)
            x_axes.append(body_x_axis_world(o.x, o.y, o.z, o.w)[:2])
            y_axes.append(body_y_axis_world(o.x, o.y, o.z, o.w)[:2])

    if not xs:
        sys.exit(f"[ERROR] no valid poses on {topic!r} (all x,y were 0)")

    return (
        np.asarray(times, dtype=np.float64),
        np.column_stack([xs, ys]),
        np.asarray(x_axes, dtype=np.float64),
        np.asarray(y_axes, dtype=np.float64),
        skipped_invalid,
    )


def resolve_cmap(name: str):
    if name in (
        "red_blue", "red-blue",
        "white_black", "white-black",
        "white_red", "white-red",
        "black_red", "black-red",
    ):
        return RED_BLUE_CMAP
    return plt.get_cmap(name)


def plot_path(
    positions: np.ndarray,
    x_axes: np.ndarray,
    y_axes: np.ndarray,
    times: np.ndarray,
    *,
    arrow_scale: float,
    arrow_stride: int,
    cmap: str,
    title: str,
    save: Path | None,
) -> None:
    fig, ax = plt.subplots(figsize=(9, 9))

    pts = positions.reshape(-1, 1, 2)
    segments = np.concatenate([pts[:-1], pts[1:]], axis=1)
    seg_t = 0.5 * (times[:-1] + times[1:])
    norm = plt.Normalize(seg_t.min(), seg_t.max())
    lc = LineCollection(
        segments, cmap=resolve_cmap(cmap), norm=norm, linewidth=1.4, zorder=2
    )
    lc.set_array(seg_t)
    ax.add_collection(lc)
    cbar = fig.colorbar(lc, ax=ax, fraction=0.046, pad=0.04)
    cbar.set_label("time [s]")
    ax.scatter(
        positions[0, 0],
        positions[0, 1],
        s=60,
        c="#16a34a",
        zorder=5,
        label="start",
    )
    ax.scatter(
        positions[-1, 0],
        positions[-1, 1],
        s=60,
        c="#dc2626",
        zorder=5,
        label="end",
    )

    idx = np.arange(0, len(positions), arrow_stride)
    quiver_kw = dict(
        angles="xy",
        scale_units="xy",
        scale=1.0 / arrow_scale,
        width=0.003,
        alpha=0.85,
    )
    ax.quiver(
        positions[idx, 0],
        positions[idx, 1],
        x_axes[idx, 0],
        x_axes[idx, 1],
        color="#f97316",
        label=f"body +X (every {arrow_stride})",
        **quiver_kw,
    )
    ax.quiver(
        positions[idx, 0],
        positions[idx, 1],
        y_axes[idx, 0],
        y_axes[idx, 1],
        color="#a855f7",
        label=f"body +Y (every {arrow_stride})",
        **quiver_kw,
    )

    x_span = float(positions[:, 0].max() - positions[:, 0].min())
    y_span = float(positions[:, 1].max() - positions[:, 1].min())
    pad = 0.05 * max(x_span, y_span, 1e-3)
    ax.set_xlim(positions[:, 0].min() - pad, positions[:, 0].max() + pad)
    ax.set_ylim(positions[:, 1].min() - pad, positions[:, 1].max() + pad)
    ax.set_aspect("equal", adjustable="box")
    ax.set_xlabel("x [m]")
    ax.set_ylabel("y [m]")
    ax.set_title(title)
    ax.grid(True, alpha=0.3)
    ax.legend(loc="best")
    fig.tight_layout()

    if save is not None:
        save.parent.mkdir(parents=True, exist_ok=True)
        fig.savefig(save, dpi=150)
        print(f"[INFO] saved {save}")
    else:
        plt.show()


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--bag",
        type=Path,
        default=DEFAULT_BAG,
        help=f"rosbag2 directory (default: {DEFAULT_BAG_NAME} in bags/)",
    )
    parser.add_argument(
        "--topic",
        default=DEFAULT_TOPIC,
        help=f"Vicon pose topic (default: {DEFAULT_TOPIC})",
    )
    parser.add_argument(
        "--arrow-scale",
        type=float,
        default=0.2,
        help="length of body-axis arrows in metres (default: 0.2)",
    )
    parser.add_argument(
        "--arrow-stride",
        type=int,
        default=80,
        help="plot body-axis arrows every N pose samples (default: 80)",
    )
    parser.add_argument(
        "--t-start",
        type=float,
        default=None,
        help="only plot poses at or after this time [s] from bag start",
    )
    parser.add_argument(
        "--t-end",
        type=float,
        default=None,
        help="only plot poses at or before this time [s] from bag start",
    )
    parser.add_argument(
        "--cmap",
        default="red_blue",
        help="colormap for path progress: red_blue or any matplotlib name",
    )
    parser.add_argument(
        "--save",
        type=Path,
        default=None,
        help="save figure to this path instead of opening a window",
    )
    args = parser.parse_args()

    bag_dir = args.bag.expanduser()
    if not bag_dir.is_absolute():
        bag_dir = BAGS_DIR / bag_dir
    bag_dir = bag_dir.resolve()
    if not bag_dir.is_dir():
        sys.exit(f"[ERROR] bag directory not found: {bag_dir}")

    times, positions, x_axes, y_axes, skipped = load_vicon_poses(
        bag_dir, topic=args.topic
    )
    if args.t_start is not None or args.t_end is not None:
        mask = np.ones(len(times), dtype=bool)
        if args.t_start is not None:
            mask &= times >= args.t_start
        if args.t_end is not None:
            mask &= times <= args.t_end
        if not np.any(mask):
            sys.exit(
                f"[ERROR] no poses in window "
                f"[{args.t_start}, {args.t_end}] s "
                f"(bag spans {times[0]:.1f}–{times[-1]:.1f} s)"
            )
        times = times[mask]
        positions = positions[mask]
        x_axes = x_axes[mask]
        y_axes = y_axes[mask]

    duration = times[-1] - times[0] if len(times) > 1 else 0.0
    window = ""
    if args.t_start is not None or args.t_end is not None:
        window = f"  window=[{args.t_start}, {args.t_end}] s"
    print(
        f"[INFO] {len(positions)} poses from {bag_dir.name}{window}\n"
        f"       topic={args.topic!r}  span={duration:.1f}s  "
        f"skipped {skipped} invalid (x=0, y=0)\n"
        f"       x range [{positions[:, 0].min():.3f}, {positions[:, 0].max():.3f}] m\n"
        f"       y range [{positions[:, 1].min():.3f}, {positions[:, 1].max():.3f}] m"
    )

    title = f"Vicon path — {bag_dir.name}"
    if args.t_start is not None or args.t_end is not None:
        t0 = args.t_start if args.t_start is not None else times[0]
        t1 = args.t_end if args.t_end is not None else times[-1]
        title += f" ({t0:.0f}–{t1:.0f} s)"

    plot_path(
        positions,
        x_axes,
        y_axes,
        times,
        arrow_scale=args.arrow_scale,
        arrow_stride=max(1, args.arrow_stride),
        cmap=args.cmap,
        title=title,
        save=args.save,
    )


if __name__ == "__main__":
    main()
