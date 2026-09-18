#!/usr/bin/env python3
"""Bounded, sequential bag replay; saves measurements, never labels them accuracy.

Run in a separate ROS_DOMAIN_ID. Each input waits for its stamped MarkerArray,
so slow processing does not silently drop frames in an offline comparison.
"""
import argparse
import copy
import json
import os
from pathlib import Path
import sqlite3
import subprocess
import time

import rclpy
from rclpy.serialization import deserialize_message
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import PointCloud2
from visualization_msgs.msg import MarkerArray


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--executable', required=True)
    parser.add_argument('--config', required=True)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--frames', type=int, default=25)
    parser.add_argument('--offset', type=int, default=0, help='Cloud message index')
    parser.add_argument('--bag-topic', default='/lidar_points')
    parser.add_argument('bags', nargs='+', type=Path)
    args = parser.parse_args()
    if args.frames < 1 or args.offset < 0:
        parser.error('frames must be positive and offset nonnegative')
    rclpy.init()
    node = rclpy.create_node('replay_check')
    pub = node.create_publisher(PointCloud2, '/replay_check/input',
        QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE))
    received = {}

    def callback(msg):
        if msg.markers:
            stamp = msg.markers[0].header.stamp
            received[(stamp.sec, stamp.nanosec)] = msg

    sub = node.create_subscription(MarkerArray, '/obstacle_detector/markers', callback, 10)
    results = []
    args.output.parent.mkdir(parents=True, exist_ok=True)
    try:
        for bag in args.bags:
            log_path = args.output.with_name(args.output.stem + '_' + bag.name + '.log')
            with log_path.open('w') as log:
                process = subprocess.Popen([
                    args.executable, '--ros-args', '--params-file', args.config,
                    '-p', 'input_topic:=/replay_check/input',
                    '-p', 'publish_debug_clouds:=false',
                ], stdout=log, stderr=subprocess.STDOUT)
                try:
                    deadline = time.monotonic() + 15
                    while pub.get_subscription_count() == 0 or node.count_publishers('/obstacle_detector/markers') == 0:
                        rclpy.spin_once(node, timeout_sec=0.05)
                        if process.poll() is not None or time.monotonic() > deadline:
                            raise RuntimeError(f'Detector did not start: {log_path}')
                    dbs = sorted(bag.expanduser().glob('*.db3'))
                    if len(dbs) != 1:
                        raise RuntimeError('This bounded checker expects one SQLite shard per bag')
                    with sqlite3.connect(f'file:{dbs[0]}?mode=ro', uri=True) as db:
                        topic = db.execute(
                            'SELECT id FROM topics WHERE name=?', (args.bag_topic,)).fetchone()
                        if topic is None:
                            raise RuntimeError(f'{args.bag_topic} is absent in {bag}')
                        rows = db.execute('SELECT data FROM messages WHERE topic_id=? ORDER BY timestamp LIMIT ? OFFSET ?',
                                          (topic[0], args.frames, args.offset))
                        frames = []
                        for index, (data,) in enumerate(rows):
                            cloud = deserialize_message(data, PointCloud2)
                            if index == 0:
                                # Discovery count can precede actual data-path readiness.
                                probe = copy.deepcopy(cloud)
                                probe.header.stamp.sec = 1
                                probe.header.stamp.nanosec = 0
                                probe.width = 0
                                probe.row_step = 0
                                probe.data = []
                                deadline = time.monotonic() + 10
                                while (1, 0) not in received:
                                    pub.publish(probe)
                                    rclpy.spin_once(node, timeout_sec=0.1)
                                    if time.monotonic() > deadline:
                                        raise RuntimeError('Warmup transport failed')
                            key = (cloud.header.stamp.sec, cloud.header.stamp.nanosec)
                            received.pop(key, None)
                            start = time.monotonic()
                            pub.publish(cloud)
                            while key not in received:
                                rclpy.spin_once(node, timeout_sec=0.02)
                                if process.poll() is not None or time.monotonic() - start > 15:
                                    raise RuntimeError(f'No output for {bag.name} frame {index}')
                            markers = received.pop(key).markers
                            centers = [m for m in markers if m.ns == 'track_center']
                            lane_markers = [m for m in markers if m.ns == 'masked_longitudinal_structure']
                            boxes = [m for m in markers if m.ns == 'candidate_boxes']
                            frames.append(dict(index=args.offset + index,
                                roundtrip_ms=1000 * (time.monotonic() - start),
                                status=[m.text for m in markers if m.ns == 'detector_status'],
                                separators=sum(m.ns == 'separator_hint' for m in markers),
                                path=[[p.x, p.y] for p in centers[0].points] if centers else [],
                                lane_offsets=[m.points[0].y - centers[0].points[0].y
                                              for m in lane_markers] if centers else [],
                                labels=[m.text for m in markers if m.ns == 'candidate_labels'],
                                boxes=[dict(x=m.pose.position.x, y=m.pose.position.y,
                                            size=[m.scale.x,m.scale.y,m.scale.z],
                                            red=m.color.g < 0.2) for m in boxes]))
                        if not frames:
                            raise RuntimeError(f'No frames at offset {args.offset}: {bag}')
                        results.append(dict(bag=bag.name, frames=frames))
                        print(bag.name, 'frames=', len(frames), 'path_valid=', sum(bool(f['path']) for f in frames),
                              'mean_boxes=', round(sum(len(f['boxes']) for f in frames)/len(frames), 2), flush=True)
                finally:
                    process.terminate()
                    try:
                        process.wait(timeout=10)
                    except subprocess.TimeoutExpired:
                        process.kill()
                        process.wait()
                    # Drain old discovery and queued samples before next process.
                    deadline = time.monotonic() + 3
                    while node.count_publishers('/obstacle_detector/markers') and time.monotonic() < deadline:
                        rclpy.spin_once(node, timeout_sec=0.05)
                    received.clear()
    finally:
        args.output.write_text(json.dumps(results, indent=2))
        node.destroy_subscription(sub)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
