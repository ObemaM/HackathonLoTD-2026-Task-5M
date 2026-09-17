#!/usr/bin/env python3
"""Inspect likely rail returns in a ROS 2 PointCloud2 bag.

The tool is intentionally read-only.  It converts the dataset axes to the
detector convention (X forward, Y left, Z up), prints a height histogram and
shows the strongest lateral peaks in several forward ranges.  It is useful for
calibrating the geometric rail estimator without guessing from an RViz view.
"""

import argparse
import sqlite3
from pathlib import Path

import numpy as np
from rclpy.serialization import deserialize_message
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2


def parse_args():
    parser = argparse.ArgumentParser()
    parser.add_argument("bag", type=Path)
    parser.add_argument("--frame", type=int, default=0)
    parser.add_argument("--rail-z-min", type=float, default=-1.75)
    parser.add_argument("--rail-z-max", type=float, default=-1.35)
    parser.add_argument("--forward-max", type=float, default=80.0)
    parser.add_argument("--lateral-limit", type=float, default=4.0)
    parser.add_argument("--plot", type=Path, help="Optional top-down PNG output")
    return parser.parse_args()


def database_path(path: Path) -> Path:
    path = path.expanduser().resolve()
    if path.is_file():
        return path
    files = sorted(path.glob("*.db3"))
    if len(files) != 1:
        raise RuntimeError(f"Expected one .db3 in {path}, found {len(files)}")
    return files[0]


def strongest_peaks(values: np.ndarray, low: float, high: float):
    edges = np.arange(low, high + 0.051, 0.05)
    counts, _ = np.histogram(values, edges)
    order = np.argsort(counts)[::-1]
    selected = []
    for index in order:
        center = 0.5 * (edges[index] + edges[index + 1])
        if all(abs(center - previous[0]) >= 0.15 for previous in selected):
            selected.append((center, int(counts[index])))
        if len(selected) == 8:
            break
    return selected


def main():
    args = parse_args()
    database = database_path(args.bag)
    connection = sqlite3.connect(f"file:{database}?mode=ro", uri=True)
    topic = connection.execute(
        "SELECT id FROM topics WHERE type='sensor_msgs/msg/PointCloud2' LIMIT 1"
    ).fetchone()
    if topic is None:
        raise RuntimeError("PointCloud2 topic not found")
    row = connection.execute(
        "SELECT data FROM messages WHERE topic_id=? ORDER BY timestamp LIMIT 1 OFFSET ?",
        (topic[0], args.frame),
    ).fetchone()
    connection.close()
    if row is None:
        raise RuntimeError(f"Frame {args.frame} not found")

    message = deserialize_message(row[0], PointCloud2)
    cloud = point_cloud2.read_points(
        message, field_names=["x", "y", "z"], skip_nans=False
    )
    sensor = np.column_stack((cloud["x"], cloud["y"], cloud["z"])).astype(np.float32)
    valid = np.isfinite(sensor).all(axis=1) & (np.square(sensor).sum(axis=1) > 0.01)
    sensor = sensor[valid]

    detector = np.column_stack((-sensor[:, 1], sensor[:, 0], sensor[:, 2]))
    roi = detector[
        (detector[:, 0] >= 2.0)
        & (detector[:, 0] <= args.forward_max)
        & (np.abs(detector[:, 1]) <= args.lateral_limit)
        & (detector[:, 2] >= -2.5)
        & (detector[:, 2] <= 0.5)
    ]

    z_edges = np.arange(-2.5, 0.501, 0.05)
    z_counts, _ = np.histogram(roi[:, 2], z_edges)
    strongest_z = np.argsort(z_counts)[::-1][:12]
    print(f"frame: {args.frame}, detector ROI points: {len(roi)}")
    print("strongest Z bands (center_m: points):")
    for index in strongest_z:
        center = 0.5 * (z_edges[index] + z_edges[index + 1])
        print(f"  {center:6.3f}: {int(z_counts[index])}")

    rail = roi[
        (roi[:, 2] >= args.rail_z_min) & (roi[:, 2] <= args.rail_z_max)
    ]
    print(
        f"rail height window [{args.rail_z_min:.2f}, {args.rail_z_max:.2f}] "
        f"contains {len(rail)} points"
    )
    ranges = ((2.0, 10.0), (10.0, 20.0), (20.0, 40.0), (40.0, args.forward_max))
    for start, end in ranges:
        values = rail[(rail[:, 0] >= start) & (rail[:, 0] < end), 1]
        peaks = strongest_peaks(values, -args.lateral_limit, args.lateral_limit)
        formatted = ", ".join(f"{position:+.2f}:{count}" for position, count in peaks)
        print(f"X [{start:4.0f}, {end:4.0f}) m: {formatted}")

    if args.plot:
        import matplotlib.pyplot as plt

        stride = max(1, len(roi) // 120_000)
        shown = roi[::stride]
        figure, axes = plt.subplots(1, 2, figsize=(12, 12), sharey=True)
        axis = axes[0]
        points = axis.scatter(
            shown[:, 1], shown[:, 0], c=shown[:, 2], s=0.7,
            cmap="turbo", vmin=-2.0, vmax=0.5,
        )
        axis.set_xlim(-args.lateral_limit, args.lateral_limit)
        axis.set_ylim(args.forward_max, 0.0)
        axis.set_aspect("equal", adjustable="box")
        axis.set_xlabel("lateral Y, m")
        axis.set_ylabel("forward X, m")
        axis.set_title(f"All ROI points, frame {args.frame}")
        figure.colorbar(points, ax=axis, label="Z, m")
        rail_axis = axes[1]
        rail_axis.scatter(rail[:, 1], rail[:, 0], c=rail[:, 2], s=1.2, cmap="viridis")
        rail_axis.set_xlim(-args.lateral_limit, args.lateral_limit)
        rail_axis.set_ylim(args.forward_max, 0.0)
        rail_axis.set_aspect("equal", adjustable="box")
        rail_axis.set_xlabel("lateral Y, m")
        rail_axis.set_title("Configured rail-height slice")
        figure.tight_layout()
        args.plot.parent.mkdir(parents=True, exist_ok=True)
        figure.savefig(args.plot, dpi=180)
        print(f"plot: {args.plot}")


if __name__ == "__main__":
    main()
