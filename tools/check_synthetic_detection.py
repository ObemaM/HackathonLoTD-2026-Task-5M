#!/usr/bin/env python3
"""End-to-end checks with artificial sparse targets, not a measured human range."""
import argparse
import json
import subprocess
import time
from pathlib import Path
import rclpy
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py.point_cloud2 import create_cloud_xyz32
from std_msgs.msg import Header
from visualization_msgs.msg import MarkerArray


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--executable', required=True)
    parser.add_argument('--config', required=True)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    rclpy.init()
    node = rclpy.create_node('synthetic_check')
    pub = node.create_publisher(PointCloud2, '/synthetic/input', QoSProfile(depth=1,
        reliability=ReliabilityPolicy.RELIABLE))
    received = {}
    sub = node.create_subscription(MarkerArray, '/obstacle_detector/markers',
        lambda msg: received.update({(msg.markers[0].header.stamp.sec,
                                      msg.markers[0].header.stamp.nanosec): msg}), 10)
    results = []
    args.output.parent.mkdir(parents=True, exist_ok=True)
    with args.output.with_suffix('.log').open('w') as log:
        process = subprocess.Popen([args.executable, '--ros-args', '--params-file', args.config,
            '-p', 'input_topic:=/synthetic/input', '-p', 'track.enabled:=false',
            '-p', 'separators.enabled:=false', '-p', 'motion.accumulation_frames:=3',
            '-p', 'publish_debug_clouds:=false'], stdout=log, stderr=subprocess.STDOUT)
        try:
            deadline = time.monotonic() + 15
            while not pub.get_subscription_count():
                rclpy.spin_once(node, timeout_sec=.05)
                if time.monotonic() > deadline or process.poll() is not None:
                    raise RuntimeError('Detector startup failed')

            def frame(sec, nanos, targets):
                header = Header()
                header.frame_id = 'synthetic_lidar'
                header.stamp.sec, header.stamp.nanosec = sec, nanos
                # Static asymmetric side structure provides registration support.
                points = [(2.0 + .25 * ((i % 7) / 7), -(3 + i * .53), -1 + j * .47)
                          for i in range(60) for j in range(9)]
                points.extend((y, -x, z) for x, y, z in targets)
                msg = create_cloud_xyz32(header, points)
                pub.publish(msg)
                deadline = time.monotonic() + 10
                while (sec, nanos) not in received:
                    rclpy.spin_once(node, timeout_sec=.05)
                    if time.monotonic() > deadline:
                        raise RuntimeError('Synthetic frame timeout')
                markers = received.pop((sec, nanos)).markers
                return [m for m in markers if m.ns == 'candidate_boxes']

            # Warm up transport with independent timestamps, then reset each case.
            for i in range(3):
                frame(1, i * 100000000, [])
            for case, distance in enumerate((10, 30, 50, 70), start=1):
                # New timestamp gap resets both tracking and accumulation.
                counts = []
                for i in range(4):
                    heights = (-.8, -.5, -.2, .1) if distance == 30 else (-.8, -.4, 0, .4)
                    boxes = frame(case * 10, i * 100000000,
                        [(distance, 0, z) for z in heights])
                    counts.append(sum(m.color.g < .2 and abs(m.pose.position.x-distance) < 1 for m in boxes))
                # At 10 m this deliberately sparse target is too sparse for the
                # 0.30 m radius; this is a negative control, not a human model.
                if distance == 10:
                    assert counts == [0, 0, 0, 0], counts
                else:
                    assert counts[:2] == [0, 0] and counts[2:] == [1, 1], (distance, counts)
                results.append(dict(distance_m=distance, confirmed_per_frame=counts))
            # Two current returns alone are below min_points. Bounded history
            # connects alternating heights; absence must remove the rendered box.
            counts = []
            for i in range(5):
                z_values = (-.8, -.4) if i % 2 == 0 else (-.6, -.2)
                boxes = frame(100, i * 100000000, [(50, 0, z) for z in z_values])
                counts.append(sum(m.color.g < .2 for m in boxes))
            assert counts[-1] == 1, counts
            assert not frame(100, 500000000, []), 'History produced ghost detection'
            results.append(dict(accumulation_confirmed=counts, ghost_after_absence=False))
            # A new bag loop must discard previously confirmed objects.
            boxes = frame(20, 0, [(50, 0, z) for z in (-.8, -.4, 0, .4)])
            assert not any(m.color.g < .2 for m in boxes), 'Bag rewind kept confirmation'
            results.append(dict(rewind_resets_confirmation=True))
            print(json.dumps(results, indent=2))
        finally:
            process.terminate()
            try:
                process.wait(timeout=10)
            except subprocess.TimeoutExpired:
                process.kill()
                process.wait()
            node.destroy_subscription(sub)
            node.destroy_node()
            rclpy.shutdown()
    args.output.write_text(json.dumps(results, indent=2))


if __name__ == '__main__':
    main()
