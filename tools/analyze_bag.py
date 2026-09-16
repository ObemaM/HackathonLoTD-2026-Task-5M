#!/usr/bin/env python3
"""Read-only PointCloud2 inventory for a ROS 2 sqlite bag.

Run this inside an environment where ROS 2 Humble is sourced. The script does
not modify the bag and deliberately samples frames by default to stay quick.
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
    parser.add_argument("bag", type=Path, help="Bag directory or its .db3 file")
    parser.add_argument(
        "--sample-every",
        type=int,
        default=10,
        help="Analyze every Nth frame (default: 10, use 1 for every frame)",
    )
    parser.add_argument(
        "--forward-axis",
        default="-y",
        choices=("x", "-x", "y", "-y", "z", "-z"),
    )
    return parser.parse_args()


def resolve_database(path: Path) -> Path:
    path = path.expanduser().resolve()
    if path.is_file() and path.suffix == ".db3":
        return path
    databases = sorted(path.glob("*.db3"))
    if len(databases) != 1:
        raise RuntimeError(f"Expected one .db3 in {path}, found {len(databases)}")
    return databases[0]


def signed_axis(values, axis):
    indices = {"x": 0, "y": 1, "z": 2}
    sign = -1.0 if axis.startswith("-") else 1.0
    return sign * values[:, indices[axis[-1]]]


def main():
    args = parse_args()
    if args.sample_every < 1:
        raise ValueError("--sample-every must be at least 1")

    database = resolve_database(args.bag)
    connection = sqlite3.connect(f"file:{database}?mode=ro", uri=True)
    topic = connection.execute(
        "SELECT id, name, type FROM topics "
        "WHERE type='sensor_msgs/msg/PointCloud2' LIMIT 1"
    ).fetchone()
    if topic is None:
        raise RuntimeError("PointCloud2 topic not found")

    topic_id, topic_name, topic_type = topic
    rows = connection.execute(
        "SELECT timestamp, data FROM messages WHERE topic_id=? ORDER BY timestamp",
        (topic_id,),
    ).fetchall()

    sampled_counts = []
    sampled_max_forward = []
    sampled_max_range = []
    frame_id = None
    fields = None

    for index, (_, blob) in enumerate(rows):
        if index % args.sample_every:
            continue
        message = deserialize_message(blob, PointCloud2)
        frame_id = message.header.frame_id
        fields = [field.name for field in message.fields]
        cloud = point_cloud2.read_points(
            message, field_names=["x", "y", "z"], skip_nans=False
        )
        xyz = np.column_stack((cloud["x"], cloud["y"], cloud["z"])).astype(
            np.float32
        )
        valid = np.isfinite(xyz).all(axis=1) & (np.square(xyz).sum(axis=1) > 0.01)
        xyz = xyz[valid]
        forward = signed_axis(xyz, args.forward_axis)
        forward = forward[forward > 0]
        ranges = np.linalg.norm(xyz, axis=1)

        sampled_counts.append(len(xyz))
        sampled_max_forward.append(float(forward.max()) if len(forward) else 0.0)
        sampled_max_range.append(float(ranges.max()) if len(ranges) else 0.0)

    connection.close()

    counts = np.asarray(sampled_counts)
    max_forward = np.asarray(sampled_max_forward)
    max_range = np.asarray(sampled_max_range)
    duration = (rows[-1][0] - rows[0][0]) / 1e9 if len(rows) > 1 else 0.0

    print(f"database: {database}")
    print(f"topic: {topic_name} ({topic_type})")
    print(f"frame_id: {frame_id}")
    print(f"fields: {fields}")
    print(f"messages: {len(rows)}")
    print(f"duration_s: {duration:.3f}")
    print(f"frequency_hz: {(len(rows) - 1) / duration:.3f}" if duration else "frequency_hz: n/a")
    print(f"sampled_frames: {len(counts)}")
    print(
        "nonzero_points min/median/max: "
        f"{counts.min()}/{int(np.median(counts))}/{counts.max()}"
    )
    print(
        "max_forward_m min/median/max: "
        f"{max_forward.min():.2f}/{np.median(max_forward):.2f}/{max_forward.max():.2f}"
    )
    print(
        "max_radial_range_m min/median/max: "
        f"{max_range.min():.2f}/{np.median(max_range):.2f}/{max_range.max():.2f}"
    )


if __name__ == "__main__":
    main()

