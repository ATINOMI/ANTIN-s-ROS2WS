"""手动建图入口：平地高度投影和完整地图包保存。"""
from collections import OrderedDict
import json
import time
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from rclpy.time import Time
from nav_msgs.msg import Odometry, OccupancyGrid
from sensor_msgs.msg import PointCloud2, LaserScan
from std_msgs.msg import String
from std_srvs.srv import Trigger
from tf2_ros import Buffer, TransformListener, StaticTransformBroadcaster, TransformBroadcaster
from .geometry import transform, collision_points, rigid, estimate_ground
from .grid import HeightGrid, GeometryStore
from .bundle import save_bundle
from .runtime import ObservationCache, ns, message_tf, tf_message, cloud_xyz, cloud_message, grid_message, positive


class MappingNode(Node):
    def __init__(self):
        super().__init__('fastlivo_height_mapper')
        self.grid = HeightGrid(positive(self, 'resolution', 0.05), positive(self, 'map_size', 20.0))
        self.geometry = GeometryStore(positive(self, 'voxel_size', 0.1))
        self.minimum = positive(self, 'min_height', 0.02)
        self.maximum = positive(self, 'max_height', 0.40)
        if self.minimum >= self.maximum:
            raise ValueError('Invalid collision height range')
        ground = self.declare_parameter('world_ground_z', -0.088).value
        if not np.isfinite(ground):
            raise ValueError('Invalid ground reference')
        self.anchor = np.eye(4)
        self.anchor[2, 3] = -ground
        self.imu_base = np.eye(4)
        self.imu_base[:3, 3] = [0.032, 0.0, -0.078]
        self.output = self.declare_parameter('output_dir', './maps/fastlivo2').value
        self.cache = ObservationCache()
        self.failed = False
        self.ground_plane = None
        self.last_time = 0
        self.scans = 0
        self.last_scan = -1
        self.pending_scans = OrderedDict()
        self.first_base = None
        self.latest_base = None
        self.buffer = Buffer()
        self.listener = TransformListener(self.buffer, self)
        self.dynamic = TransformBroadcaster(self)
        self.static = StaticTransformBroadcaster(self)
        self.static.sendTransform(tf_message('map', 'camera_init', self.get_clock().now().to_msg(), self.anchor))
        sensor = QoSProfile(depth=5, reliability=ReliabilityPolicy.BEST_EFFORT)
        latched = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.map_pub = self.create_publisher(OccupancyGrid, '/fastlivo/map_2d_live', latched)
        self.collision_pub = self.create_publisher(PointCloud2, '/fastlivo/collision_cloud', sensor)
        self.status_pub = self.create_publisher(String, '/fastlivo/mapping_status', latched)
        self.create_subscription(Odometry, '/aft_mapped_to_init', self.pose, sensor)
        self.create_subscription(PointCloud2, '/fastlivo/cloud_world', self.cloud, sensor)
        self.create_subscription(LaserScan, '/scan', self.scan, sensor)
        self.create_service(Trigger, '/fastlivo/save_nav_map', self.save)
        self.create_timer(0.5, self.publish)

    def check_time(self):
        current = self.get_clock().now().nanoseconds
        if current < self.last_time:
            self.failed = True
        self.last_time = current

    def pose(self, message):
        try:
            if ns(message.header.stamp) < self.cache.last_processed:
                self.failed = True
                return
            self.cache.pose(message)
            self.consume()
        except ValueError as error:
            self.failed = True
            self.get_logger().error(str(error))

    def cloud(self, message):
        try:
            if ns(message.header.stamp) < self.cache.last_processed:
                self.failed = True
                return
            self.cache.cloud(message)
            self.consume()
        except ValueError as error:
            self.failed = True
            self.get_logger().error(str(error))

    def consume(self):
        self.check_time()
        if self.failed:
            return
        item = self.cache.take()
        if item is None:
            return
        cloud, _, world_imu = item
        if abs(self.get_clock().now().nanoseconds - ns(cloud.header.stamp)) > 1000000000:
            return
        points = transform(cloud_xyz(cloud), self.anchor)
        base = self.anchor @ world_imu @ self.imu_base
        own = np.linalg.norm(points[:, :2] - base[:2, 3], axis=1) < 0.28
        self.geometry.add(points[~own])
        try:
            self.ground_plane = estimate_ground(points, base)
        except ValueError:
            self.ground_plane = None
            return
        collisions = collision_points(points, base, self.minimum, self.maximum, ground_plane=self.ground_plane)
        self.grid.mark(collisions)
        self.latest_base = base
        if self.first_base is None:
            self.first_base = base.copy()
        try:
            from .geometry import planar
            odom_base = message_tf(self.buffer.lookup_transform('odom', 'base_footprint', Time.from_msg(cloud.header.stamp)))
            self.dynamic.sendTransform(tf_message('map', 'odom', cloud.header.stamp, planar(base) @ np.linalg.inv(planar(odom_base))))
        except Exception:
            pass
        self.collision_pub.publish(cloud_message(collisions, 'map', cloud.header.stamp))
        for pending in list(self.pending_scans.values()):
            self.integrate_scan(pending)

    def scan(self, message):
        self.pending_scans[ns(message.header.stamp)] = message
        while len(self.pending_scans) > 20:
            self.pending_scans.popitem(last=False)
        self.integrate_scan(message)

    def integrate_scan(self, message):
        self.check_time()
        stamp = ns(message.header.stamp)
        if self.failed or stamp <= self.last_scan:
            return
        world_imu = self.cache.closest(stamp)
        if world_imu is None or abs(self.get_clock().now().nanoseconds - stamp) > 1000000000:
            return
        if not message.header.frame_id or not np.isfinite([message.angle_min, message.angle_increment, message.range_min, message.range_max]).all() or message.range_max <= message.range_min:
            return
        try:
            base_scan = message_tf(self.buffer.lookup_transform('base_footprint', message.header.frame_id, Time.from_msg(message.header.stamp)))
        except Exception:
            return
        map_base = self.anchor @ world_imu @ self.imu_base
        map_scan = map_base @ base_scan
        endpoints, hits = [], []
        for index, distance in enumerate(message.ranges):
            no_return = np.isposinf(distance)
            if not no_return and (not np.isfinite(distance) or distance < message.range_min or distance > message.range_max):
                continue
            length = min(3.0, message.range_max if no_return else distance)
            angle = message.angle_min + index * message.angle_increment
            endpoint = transform([[length * np.cos(angle), length * np.sin(angle), 0]], map_scan)[0]
            endpoints.append(endpoint[:2])
            if not no_return and distance <= 3.0 and np.linalg.norm(endpoint[:2] - map_base[:2, 3]) > 0.28:
                hits.append(endpoint)
        if endpoints:
            self.grid.rays(map_scan[:2, 3], endpoints)
            if hits:
                self.grid.mark(np.array(hits))
            self.scans += 1
            self.last_scan = stamp
            for key in list(self.pending_scans):
                if key <= stamp:
                    del self.pending_scans[key]

    def publish(self):
        self.check_time()
        self.map_pub.publish(grid_message(self.grid, self.get_clock().now().to_msg()))
        state = {'valid': not self.failed and not self.grid.out_of_bounds and not self.geometry.full and self.ground_plane is not None,
                 'geometry_points': len(self.geometry.voxels), 'scan_observations': self.scans,
                 'free_cells': int((self.grid.data() == 0).sum()), 'reason': 'reset_detected' if self.failed else 'flat_scan_model',
                 'last_livo_stamp': self.cache.last_processed, 'last_scan_stamp': self.last_scan}
        self.status_pub.publish(String(data=json.dumps(state)))

    def save(self, request, response):
        if self.failed or self.latest_base is None or self.ground_plane is None or not self.scans:
            response.success, response.message = False, 'No healthy paired geometry and scan observations'
            return response
        alignment = {'map_world': self.anchor.tolist(), 'imu_base': self.imu_base.tolist(),
                     'min_height': self.minimum, 'max_height': self.maximum,
                     'last_map_base': self.latest_base.tolist(), 'frontend': 'FAST-LIVO2 RDR 837b7bb + local patches',
                     'world_ground_z': -float(self.anchor[2, 3]), 'scan_observations': self.scans,
                     'first_map_base': self.first_base.tolist(), 'ground_plane': self.ground_plane.tolist()}
        try:
            path = save_bundle(self.output, self.grid, self.geometry, alignment)
            response.success, response.message = True, str(path)
            self.get_logger().info('Saved navigation bundle: ' + str(path))
        except (ValueError, OSError) as error:
            response.success, response.message = False, str(error)
        return response


def main():
    import signal
    from rclpy.signals import SignalHandlerOptions
    stop = [False]
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    signal.signal(signal.SIGINT, lambda *_: stop.__setitem__(0, True))
    signal.signal(signal.SIGTERM, lambda *_: stop.__setitem__(0, True))
    node = MappingNode()
    try:
        while rclpy.ok() and not stop[0]:
            rclpy.spin_once(node, timeout_sec=0.05)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
