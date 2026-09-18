#!/usr/bin/env python3
"""Summarize bounded replay and plot paths against measured rail-height returns."""
import argparse
import json
from pathlib import Path
import sqlite3
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from rclpy.serialization import deserialize_message
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--results', type=Path, default=Path('experiments/results'))
    parser.add_argument('--data', type=Path, default=Path.home() / 'HackathonLoDT/data/for_hackathon')
    parser.add_argument('--after-name', default='after')
    args = parser.parse_args()
    summary = []
    figure, axes = plt.subplots(5, 3, figsize=(15, 20))
    for column, offset in enumerate((0, 120, 240)):
        before = json.loads((args.results / f'before_{offset}.json').read_text())
        after = json.loads((args.results / f'{args.after_name}_{offset}.json').read_text())
        for row, (old, new) in enumerate(zip(before, after)):
            assert old['bag'] == new['bag']
            assert [f['index'] for f in old['frames']] == [f['index'] for f in new['frames']]
            item = dict(bag=new['bag'], offset=offset, frames=len(new['frames']))
            for name, record in [('before', old), ('after', new)]:
                frames = record['frames']
                lengths = [f['path'][-1][0] if f['path'] else 0 for f in frames]
                item[name] = dict(valid=sum(bool(f['path']) for f in frames),
                    median_trusted_forward_m=float(np.median(lengths)),
                    median_roundtrip_ms=float(np.median([f['roundtrip_ms'] for f in frames])),
                    mean_displayed_boxes=float(np.mean([len(f['boxes']) for f in frames])),
                    mean_confirmed_boxes=float(np.mean([
                        sum(box.get('red', False) for box in f['boxes']) for f in frames])),
                    motion_valid=sum(any('motion=valid' in s for s in f.get('status', [])) for f in frames),
                    separator_frames=sum(f.get('separators', 0) > 0 for f in frames))
            summary.append(item)
            index = len(new['frames']) // 2
            frame_index = new['frames'][index]['index']
            dbpath = next((args.data / new['bag']).glob('*.db3'))
            with sqlite3.connect(f'file:{dbpath}?mode=ro', uri=True) as db:
                topic = db.execute("SELECT id FROM topics WHERE name='/lidar_points'").fetchone()[0]
                data = db.execute('SELECT data FROM messages WHERE topic_id=? ORDER BY timestamp LIMIT 1 OFFSET ?',
                                  (topic, frame_index)).fetchone()[0]
            msg = deserialize_message(data, PointCloud2)
            points = point_cloud2.read_points(msg, field_names=['x','y','z'], skip_nans=True)
            x, y, z = -points['y'], points['x'], points['z']
            mask = (x > 2) & (x < 90) & (abs(y) < 5) & (z > -1.75) & (z < -1.15)
            axis = axes[row, column]
            axis.scatter(x[mask], y[mask], s=.3, c='gray', alpha=.6)
            for name, record, color in [('before', old, 'orange'), ('after', new, 'blue')]:
                path = np.array(record['frames'][index]['path'])
                if len(path):
                    axis.plot(path[:,0], path[:,1], color=color, label=name)
            axis.set(xlim=(0,90), ylim=(-5,5), xlabel='Forward, m', ylabel='Lateral, m',
                     title=f"{new['bag']}\nframe {frame_index}")
            axis.legend(fontsize=7)
            axis.grid(alpha=.2)
    figure.tight_layout()
    figure.savefig(args.results / f'path_comparison_{args.after_name}.png', dpi=130)
    (args.results / f'comparison_summary_{args.after_name}.json').write_text(json.dumps(summary, indent=2))
    for item in summary:
        print(item['bag'], item['offset'], 'n=', item['frames'],
              'range=', item['before']['median_trusted_forward_m'], '->', item['after']['median_trusted_forward_m'],
              'motion=', item['after']['motion_valid'], 'separator=', item['after']['separator_frames'])


if __name__ == '__main__':
    main()
