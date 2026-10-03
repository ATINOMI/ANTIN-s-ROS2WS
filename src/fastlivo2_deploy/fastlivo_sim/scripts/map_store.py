#!/usr/bin/env python3
"""Retain final-state 3D scan geometry and checkpoint a per-run voxel map."""
from datetime import datetime
import json
import math
import os
import signal
from pathlib import Path
import tempfile

import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.signals import SignalHandlerOptions
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from sensor_msgs.msg import PointCloud2, PointField
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Header
from std_srvs.srv import Trigger


class VoxelMap:
    def __init__(self, resolution=0.1, max_voxels=1000000):
        if not math.isfinite(resolution) or resolution <= 0 or max_voxels < 1:
            raise ValueError('Positive finite voxel resolution and capacity required')
        self.resolution = resolution
        self.max_voxels = max_voxels
        self.cells = {}
        self.capacity_reached = False

    def add(self, points):
        points = np.asarray(points, dtype=np.float32).reshape(-1, 4)
        points = points[np.isfinite(points).all(axis=1)]
        if not len(points):
            return
        scaled = points[:, :3].astype(np.float64) / self.resolution
        valid = (np.abs(scaled) < 9e18).all(axis=1)
        points, scaled = points[valid], scaled[valid]
        keys = np.floor(scaled).astype(np.int64)
        keys, indices = np.unique(keys, axis=0, return_index=True)
        for key, point in zip(keys, points[indices]):
            key = tuple(key)
            if key in self.cells:
                continue
            if len(self.cells) >= self.max_voxels:
                self.capacity_reached = True
                continue
            self.cells[key] = point.copy()

    def points(self):
        return np.asarray(list(self.cells.values()), dtype=np.float32).reshape(-1, 4)


def read_color_points(message):
    """Decode packed RGB bits before voxel filtering, including NaN-like alpha bits."""
    fields = {field.name: field.datatype for field in message.fields}
    if fields.get('rgb') not in (PointField.FLOAT32, PointField.UINT32):
        raise ValueError('Expected packed float32 or uint32 rgb in the camera-colored cloud')
    records = point_cloud2.read_points(message, field_names=['x', 'y', 'z', 'rgb'])
    packed = records['rgb']
    if fields['rgb'] == PointField.FLOAT32:
        packed = packed.view(np.uint32)
    # A 24-bit RGB integer is exactly representable in float32; alpha is not map color.
    return np.column_stack([records['x'], records['y'], records['z'], packed & 0xFFFFFF]).astype(np.float32)


def cloud_array(points, colored=False):
    points = np.array(points, dtype='<f4', copy=True).reshape(-1, 4)
    if colored:
        points.view('<u4')[:, 3] = points[:, 3].astype(np.uint32) & 0xFFFFFF
    return points


def save_pcd(path, points, colored=False):
    """Replace this session's checkpoint atomically; keep other sessions intact."""
    points = cloud_array(points, colored)
    channel = 'rgb' if colored else 'intensity'
    header = (f'# .PCD v0.7\nVERSION 0.7\nFIELDS x y z {channel}\nSIZE 4 4 4 4\n'
              f'TYPE F F F F\nCOUNT 1 1 1 1\nWIDTH {len(points)}\nHEIGHT 1\n'
              f'VIEWPOINT 0 0 0 1 0 0 0\nPOINTS {len(points)}\nDATA binary\n')
    temporary = path.with_suffix('.pcd.tmp')
    with temporary.open('wb') as stream:
        stream.write(header.encode('ascii'))
        stream.write(points.tobytes())
        stream.flush()
        os.fsync(stream.fileno())
    os.replace(temporary, path)


class MapStore(Node):
    def __init__(self):
        super().__init__('fastlivo_map_store')
        self.colored = self.declare_parameter('colored', False).value
        self.channel = 'rgb' if self.colored else 'intensity'
        self.filename = 'color_map.pcd' if self.colored else 'map.pcd'
        self.colors_unavailable = False
        output = self.declare_parameter('output_dir', './maps/fastlivo2').value
        resolution = self.declare_parameter('voxel_size', 0.1).value
        capacity = self.declare_parameter('max_voxels', 1000000).value
        period = self.declare_parameter('save_period', 15.0).value
        if not math.isfinite(period) or period <= 0:
            raise ValueError('save_period must be positive and finite')
        self.map = VoxelMap(resolution, capacity)
        self.frame = 'camera_init'
        self.stamp = None
        self.last_ns = -1
        self.first_ns = None
        self.reset_detected = False
        self.scans = 0
        root = Path(output).expanduser().resolve()
        root.mkdir(parents=True, exist_ok=True)
        self.directory = Path(tempfile.mkdtemp(prefix=datetime.now().strftime('%Y%m%d_%H%M%S_') + ('color_' if self.colored else ''), dir=root))
        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        topic = '/fastlivo/color_map' if self.colored else '/fastlivo/map'
        source = '/cloud_registered' if self.colored else '/fastlivo/cloud_world'
        service = '/fastlivo/save_color_map' if self.colored else '/fastlivo/save_map'
        self.publisher = self.create_publisher(PointCloud2, topic, qos)
        self.subscription = self.create_subscription(
            PointCloud2, source, self.receive,
            QoSProfile(depth=2, reliability=ReliabilityPolicy.BEST_EFFORT))
        self.service = self.create_service(Trigger, service, self.save_service)
        self.publish_timer = self.create_timer(1.0, self.publish_map)
        self.save_timer = self.create_timer(period, self.checkpoint)
        self.get_logger().info(f'Map checkpoints: {self.directory / self.filename}')

    def receive(self, message):
        ns = message.header.stamp.sec * 1000000000 + message.header.stamp.nanosec
        if self.reset_detected:
            return
        if message.header.frame_id != self.frame or ns < self.last_ns:
            self.reset_detected = True
            self.checkpoint()
            self.get_logger().error('Map frame/time changed; retained old map and stopped merging. Restart the whole deployment.')
            return
        if ns == self.last_ns:
            return
        if self.colored:
            try:
                points = read_color_points(message)
            except ValueError as error:
                if not self.colors_unavailable:
                    self.get_logger().warning(str(error))
                    self.colors_unavailable = True
                return
        else:
            points = point_cloud2.read_points_numpy(message, field_names=['x', 'y', 'z', 'intensity'])
        self.map.add(points)
        self.scans += 1
        self.last_ns = ns
        if self.first_ns is None:
            self.first_ns = ns
        self.stamp = message.header.stamp
        if self.map.capacity_reached and self.scans % 100 == 0:
            self.get_logger().warning('Voxel capacity reached; existing map retained, new cells rejected')

    def publish_map(self):
        if self.stamp is None or not self.map.cells:
            return
        fields = [PointField(name=name, offset=i * 4, datatype=PointField.FLOAT32, count=1)
                  for i, name in enumerate(['x', 'y', 'z', self.channel])]
        message = point_cloud2.create_cloud(Header(stamp=self.stamp, frame_id=self.frame), fields, cloud_array(self.map.points(), self.colored))
        self.publisher.publish(message)

    def checkpoint(self):
        if not self.map.cells:
            return False, 'No valid map points yet'
        try:
            points = self.map.points()
            save_pcd(self.directory / self.filename, points, self.colored)
            metadata = {'frame_id': self.frame, 'coordinate_scope': 'relative to this FAST-LIVO initialization',
                        'voxel_size': self.map.resolution, 'points': len(points), 'scans': self.scans,
                        'first_stamp_ns': self.first_ns, 'last_stamp_ns': self.last_ns,
                        'capacity_reached': self.map.capacity_reached, 'reset_detected': self.reset_detected,
                        'prior_map_localization': False, 'colored': self.colored,
                        'fields': ['x', 'y', 'z', self.channel], 'file': self.filename}
            temporary = self.directory / 'metadata.json.tmp'
            temporary.write_text(json.dumps(metadata, indent=2) + '\n')
            os.replace(temporary, self.directory / 'metadata.json')
            self.get_logger().info(f'Saved {len(points)} map points from {self.scans} scans')
            return True, str(self.directory / self.filename)
        except OSError as error:
            self.get_logger().error(f'Map checkpoint failed: {error}')
            return False, str(error)

    def save_service(self, request, response):
        response.success, response.message = self.checkpoint()
        return response


def main():
    stop_requested = False

    def request_stop(signum, frame):
        nonlocal stop_requested
        stop_requested = True

    # Finish the active callback and checkpoint before invalidating the ROS context.
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    signal.signal(signal.SIGINT, request_stop)
    signal.signal(signal.SIGTERM, request_stop)
    node = MapStore()
    try:
        while rclpy.ok() and not stop_requested:
            rclpy.spin_once(node, timeout_sec=0.1)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.checkpoint()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
