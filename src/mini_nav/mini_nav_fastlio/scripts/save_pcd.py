#!/usr/bin/env python3
"""Save one current FAST-LIO2 map message as a binary XYZ-intensity PCD."""
import argparse
from datetime import datetime
import hashlib
import json
import os
from pathlib import Path
import tempfile
import time

import numpy as np
import rclpy
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--output-dir', type=Path, required=True)
    parser.add_argument('--topic', default='/ikd_tree')
    parser.add_argument('--timeout', type=float, default=10.0)
    args = parser.parse_args()
    if not np.isfinite(args.timeout) or args.timeout <= 0:
        parser.error('--timeout must be a finite positive number')
    rclpy.init()
    node = rclpy.create_node('scurm_pcd_snapshot')
    messages = []
    subscription = node.create_subscription(PointCloud2, args.topic,
        lambda msg: messages.append(msg) if not messages else None, qos_profile_sensor_data)
    try:
        deadline = time.monotonic() + args.timeout
        while not messages and time.monotonic() < deadline and rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.1)
        if not messages:
            raise RuntimeError(f'No map received from {args.topic}; check that FAST-LIO2 is running')
        msg = messages[0]
        names = {field.name for field in msg.fields}
        if not {'x', 'y', 'z'}.issubset(names):
            raise RuntimeError('Map does not contain XYZ fields')
        fields = ('x', 'y', 'z', 'intensity') if 'intensity' in names else ('x', 'y', 'z')
        values = point_cloud2.read_points_numpy(msg, field_names=fields, skip_nans=True)
        values = np.asarray(values).reshape(-1, len(fields))
        values = values[np.isfinite(values).all(axis=1)]
        if not len(values):
            raise RuntimeError('Map contains no finite points; no PCD written')
        if len(fields) == 3:
            values = np.column_stack((values, np.zeros(len(values))))
        values = np.ascontiguousarray(values, dtype='<f4')
        count = len(values)
        header = ('# .PCD v0.7 - Point Cloud Data file format\n'
                  'VERSION 0.7\nFIELDS x y z intensity\nSIZE 4 4 4 4\n'
                  'TYPE F F F F\nCOUNT 1 1 1 1\n'
                  f'WIDTH {count}\nHEIGHT 1\nVIEWPOINT 0 0 0 1 0 0 0\n'
                  f'POINTS {count}\nDATA binary\n').encode('ascii')
        created = datetime.now().astimezone()
        folder = args.output_dir.expanduser().resolve() / created.strftime('map_%Y%m%d_%H%M%S_%f')
        folder.mkdir(parents=True, exist_ok=False)
        destination = folder / 'map.pcd'
        temporary = None
        try:
            with tempfile.NamedTemporaryFile(dir=folder, suffix='.tmp', delete=False) as output:
                temporary = Path(output.name)
                output.write(header)
                output.write(values.tobytes())
                output.flush()
                os.fsync(output.fileno())
            # Atomic publication without replacing any existing map.
            os.link(temporary, destination)
        finally:
            if temporary is not None:
                temporary.unlink(missing_ok=True)
        metadata = {
            'created_at': created.isoformat(), 'source_topic': args.topic,
            'frame_id': msg.header.frame_id,
            'stamp': {'sec': msg.header.stamp.sec, 'nanosec': msg.header.stamp.nanosec},
            'source_points': msg.width * msg.height, 'saved_points': count,
            'fields': ['x', 'y', 'z', 'intensity'], 'format': 'PCD v0.7 binary little-endian',
            'bounds_min_xyz': values[:, :3].min(axis=0).tolist(),
            'bounds_max_xyz': values[:, :3].max(axis=0).tolist(),
            'sha256': hashlib.sha256(destination.read_bytes()).hexdigest(),
        }
        (folder / 'metadata.json').write_text(json.dumps(metadata, indent=2) + '\n')
        print(f'Saved {count} points in frame {msg.header.frame_id}: {destination}')
        print(f'Metadata: {folder / "metadata.json"}')
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
