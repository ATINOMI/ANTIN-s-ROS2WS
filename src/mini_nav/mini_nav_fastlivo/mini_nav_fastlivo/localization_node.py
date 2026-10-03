"""只读先验地图定位：人工初值、持续 GICP 约束与动态 TF。"""
from concurrent.futures import ThreadPoolExecutor
import json
import time
import uuid
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from rclpy.time import Time
from rclpy.duration import Duration
from nav_msgs.msg import Odometry
from sensor_msgs.msg import PointCloud2, Imu, Image
from geometry_msgs.msg import PoseWithCovarianceStamped
from std_msgs.msg import String, Bool
from tf2_ros import Buffer, TransformListener, TransformBroadcaster
from .bundle import load_bundle
from .geometry import rigid, planar, transform, collision_points, estimate_ground
from .registration import PriorMatcher
from .runtime import ObservationCache, ns, message_pose, message_tf, tf_message, cloud_xyz, cloud_message, positive


class LocalizationNode(Node):
    def __init__(self):
        super().__init__('fastlivo_prior_localizer')
        root = self.declare_parameter('map_bundle', '').value
        self.root, self.manifest, self.alignment, points = load_bundle(root)
        self.matcher = PriorMatcher(points)
        self.imu_base = rigid(self.alignment['imu_base'])
        self.minimum, self.maximum = self.alignment['min_height'], self.alignment['max_height']
        self.input_timeout = positive(self, 'input_timeout', 0.8)
        self.match_timeout = positive(self, 'match_timeout', 2.0)
        self.transform_tolerance = positive(self, 'transform_tolerance', 0.20)
        if self.transform_tolerance > 0.5:
            raise ValueError('TF extrapolation must be bounded by 0.5 seconds')
        self.cache = ObservationCache()
        self.buffer = Buffer()
        self.listener = TransformListener(self.buffer, self)
        self.broadcaster = TransformBroadcaster(self)
        self.pool = ThreadPoolExecutor(max_workers=1)
        self.future = None
        self.seed = None
        self.correction = None
        self.observation = None
        self.observed_at = 0.0
        self.matched_at = 0.0
        self.submitted_at = 0.0
        self.latest_clock = -1
        self.clock_advanced_at = time.monotonic()
        self.last_pose_stamp = -1
        self.last_cloud_stamp = -1
        self.received = {}
        self.diagnostics = {}
        self.reason = 'waiting_initial_pose'
        self.session = uuid.uuid4().hex
        self.generation = 0
        sensor = QoSProfile(depth=5, reliability=ReliabilityPolicy.BEST_EFFORT)
        live = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE)
        latched = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.quality = self.create_publisher(Bool, '/mini_nav/localization_valid', live)
        self.state_pub = self.create_publisher(String, '/mini_nav/localization_state', live)
        self.epoch_pub = self.create_publisher(String, '/mini_nav/localization_epoch', latched)
        self.prior_pub = self.create_publisher(PointCloud2, '/fastlivo/prior_map', latched)
        self.prior_pub.publish(cloud_message(points, 'map', self.get_clock().now().to_msg()))
        self.collision_pub = self.create_publisher(PointCloud2, '/fastlivo/collision_cloud', sensor)
        self.create_subscription(Odometry, '/aft_mapped_to_init', self.pose, sensor)
        self.create_subscription(PointCloud2, '/fastlivo/cloud_world', self.cloud, sensor)
        self.create_subscription(Imu, '/imu', lambda msg: self.sensor('imu', msg), sensor)
        self.create_subscription(Image, '/camera/image_raw', lambda msg: self.sensor('image', msg), sensor)
        self.create_subscription(PoseWithCovarianceStamped, '/initialpose', self.initial_pose, 10)
        self.create_timer(0.05, self.tick)
        self.publish_epoch()

    def publish_epoch(self):
        self.epoch_pub.publish(String(data=self.manifest['map_id'] + '/' + self.session + '/' + str(self.generation)))

    def invalidate(self, reason):
        if self.correction is not None or self.seed is not None:
            self.generation += 1
            self.publish_epoch()
        self.correction = self.seed = None
        self.matched_at = 0.0
        self.reason = reason
        self.quality.publish(Bool(data=False))

    def sensor(self, name, message):
        self.received[name] = (time.monotonic(), ns(message.header.stamp))

    def pose(self, message):
        stamp = ns(message.header.stamp)
        if stamp < self.last_pose_stamp:
            self.invalidate('frontend_reset')
            self.cache = ObservationCache()
        self.last_pose_stamp = stamp
        try:
            matrix = message_pose(message.pose.pose)
            if self.observation is not None and np.linalg.norm(matrix[:3, 3] - self.observation[1][:3, 3]) > 0.5:
                self.invalidate('frontend_pose_jump')
            self.cache.pose(message)
            self.received['pose'] = (time.monotonic(), stamp)
            self.consume()
        except ValueError:
            self.invalidate('invalid_pose')

    def cloud(self, message):
        stamp = ns(message.header.stamp)
        if stamp < self.last_cloud_stamp:
            self.invalidate('cloud_reset')
            self.cache = ObservationCache()
        self.last_cloud_stamp = stamp
        try:
            self.cache.cloud(message)
            self.received['cloud'] = (time.monotonic(), stamp)
            self.consume()
        except ValueError:
            self.invalidate('invalid_cloud')

    def consume(self):
        item = self.cache.take()
        if item is None:
            return
        cloud, _, world_imu = item
        self.observation = (cloud.header.stamp, world_imu, cloud_xyz(cloud))
        self.observed_at = time.monotonic()
        if self.correction is not None:
            self.publish_transforms()
        self.submit()

    def initial_pose(self, message):
        self.invalidate('initial_pose_requested')
        if message.header.frame_id != 'map' or self.observation is None:
            self.reason = 'initial_pose_without_observation'
            return
        try:
            desired = message_pose(message.pose.pose)
            # 2D 初值的高度取保存的平地车体参考，不依赖 Gazebo 真值。
            desired[2, 3] = self.alignment['last_map_base'][2][3]
            current = self.observation[1] @ self.imu_base
            self.seed = desired @ np.linalg.inv(current)
            self.reason = 'aligning'
            self.submit(force=True)
        except ValueError:
            self.invalidate('invalid_initial_pose')

    def submit(self, force=False):
        now = time.monotonic()
        initial = self.correction if self.correction is not None else self.seed
        if initial is None or self.observation is None or self.future is not None or (not force and now - self.submitted_at < 0.5):
            return
        points = self.observation[2]
        world_base = self.observation[1] @ self.imu_base
        good = np.linalg.norm(points[:, :2] - world_base[:2, 3], axis=1) > 0.28
        self.future = (self.pool.submit(self.matcher.align, points[good].copy(), initial.copy()),
                       self.generation, self.observation[0], now, initial.copy())
        self.submitted_at = now

    def publish_transforms(self):
        if self.correction is None or self.observation is None:
            return False
        stamp, world_imu, points = self.observation
        try:
            odom_base = message_tf(self.buffer.lookup_transform('odom', 'base_footprint', Time.from_msg(stamp)))
            map_base = self.correction @ world_imu @ self.imu_base
            map_odom = planar(map_base) @ np.linalg.inv(planar(odom_base))
            # 与 AMCL 相同的有限 TF 容忍；变换仍由同一测量时刻的两套位姿计算。
            expiration = (Time.from_msg(stamp) + Duration(seconds=self.transform_tolerance)).to_msg()
            self.broadcaster.sendTransform([tf_message('map', 'odom', expiration, map_odom),
                                             tf_message('map', 'camera_init', expiration, self.correction)])
            points_map = transform(points, self.correction)
            plane = estimate_ground(points_map, map_base)
            collision_map = collision_points(points_map, map_base, self.minimum, self.maximum, ground_plane=plane)
            collision_odom = transform(collision_map, np.linalg.inv(map_odom))
            self.collision_pub.publish(cloud_message(collision_odom, 'odom', stamp))
            return True
        except Exception:
            self.reason = 'waiting_same_stamp_odom'
            return False

    def tick(self):
        steady = time.monotonic()
        clock = self.get_clock().now().nanoseconds
        if clock < self.latest_clock:
            self.invalidate('clock_rewind')
        if clock != self.latest_clock:
            self.clock_advanced_at = steady
        self.latest_clock = clock
        if self.future is not None and self.future[0].done():
            future, generation, stamp, started, initial = self.future
            self.future = None
            if generation == self.generation:
                try:
                    correction, diagnostics = future.result()
                    self.diagnostics = diagnostics
                    delta = correction @ np.linalg.inv(initial)
                    jump = np.linalg.norm(delta[:3, 3])
                    angle = np.arccos(np.clip((np.trace(delta[:3, :3]) - 1) / 2, -1, 1))
                    if not diagnostics['valid'] or steady - started > self.match_timeout or (self.correction is not None and (jump > 0.30 or angle > 0.20)):
                        self.invalidate('map_match_rejected')
                    else:
                        self.correction = correction
                        self.seed = None
                        self.matched_at = steady
                        self.reason = 'localized'
                except Exception as error:
                    self.get_logger().warning('Registration rejected: ' + str(error))
                    self.invalidate('registration_error')
        fresh = all(name in self.received and steady - self.received[name][0] < self.input_timeout and
                    -0.1 <= (clock - self.received[name][1]) / 1e9 < self.input_timeout
                    for name in ['pose', 'cloud', 'imu', 'image'])
        paired_fresh = self.observation is not None and steady - self.observed_at < self.input_timeout and -0.1 <= (clock - ns(self.observation[0])) / 1e9 < self.input_timeout
        healthy = fresh and paired_fresh and steady - self.clock_advanced_at < 0.35 and self.correction is not None and steady - self.matched_at < self.match_timeout
        if self.correction is not None and not healthy:
            self.invalidate('input_or_constraint_timeout')
        tf_ok = healthy and self.publish_transforms()
        self.quality.publish(Bool(data=bool(tf_ok)))
        self.state_pub.publish(String(data=json.dumps({'schema': 1, 'map_id': self.manifest['map_id'],
            'session_id': self.session, 'generation': self.generation, 'valid': bool(tf_ok),
            'reason': self.reason, 'estimate_stamp_ns': ns(self.observation[0]) if self.observation else 0,
            'tf_tolerance': self.transform_tolerance, 'map_read_only': True,
            **{key: value for key, value in self.diagnostics.items() if key != 'valid'}})))
        if self.correction is not None:
            self.submit()

    def destroy_node(self):
        self.quality.publish(Bool(data=False))
        self.pool.shutdown(wait=True, cancel_futures=True)
        return super().destroy_node()


def main():
    import signal
    from rclpy.signals import SignalHandlerOptions
    stop = [False]
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    signal.signal(signal.SIGINT, lambda *_: stop.__setitem__(0, True))
    signal.signal(signal.SIGTERM, lambda *_: stop.__setitem__(0, True))
    node = LocalizationNode()
    try:
        while rclpy.ok() and not stop[0]:
            rclpy.spin_once(node, timeout_sec=0.05)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
