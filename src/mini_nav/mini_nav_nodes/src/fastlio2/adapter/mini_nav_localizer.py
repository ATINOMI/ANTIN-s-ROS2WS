#!/usr/bin/env python3
"""Adapt SCURM's fixed-prior FAST-LIO2 estimator to mini_nav localization."""
import copy
from collections import deque
import json
import math
from pathlib import Path
import time
import uuid

import numpy as np
from scipy.spatial.transform import Rotation
import rclpy
from rclpy.clock import Clock, ClockType
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from rclpy.qos import QoSProfile, DurabilityPolicy, qos_profile_sensor_data
from diagnostic_msgs.msg import DiagnosticArray
from geometry_msgs.msg import PoseWithCovarianceStamped, TransformStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu, PointCloud2
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Bool, String, Header
from tf2_ros import TransformBroadcaster

from localization_geometry import IMU_BASE, IMU_LIDAR, interpolate, match_valid, planar, pose_matrix


def seconds(stamp):
    return stamp.sec + stamp.nanosec * 1e-9


def write_pose(pose, matrix):
    pose.position.x, pose.position.y, pose.position.z = map(float, matrix[:3, 3])
    q = Rotation.from_matrix(matrix[:3, :3]).as_quat()
    pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w = map(float, q)


class MiniNavLocalizer(Node):
    def __init__(self):
        super().__init__('scurm_mini_nav_localizer')
        self.minimum_points = self.declare_parameter('minimum_points', 60).value
        self.minimum_ratio = self.declare_parameter('minimum_ratio', .2).value
        self.maximum_residual = self.declare_parameter('maximum_residual', .08).value
        self.timeout = self.declare_parameter('data_timeout', .5).value
        prior_path = self.declare_parameter('prior_map_path', '').value
        if (self.minimum_points < 30 or not 0 < self.minimum_ratio <= 1 or
                not math.isfinite(self.maximum_residual) or not 0 < self.maximum_residual <= .15 or
                not math.isfinite(self.timeout) or not 0 < self.timeout <= 1):
            raise ValueError('Invalid localization quality limits')
        data = Path(prior_path).read_bytes().split(b'DATA binary\n', 1)[1]
        points = np.frombuffer(data, dtype='<f4').reshape(-1, 4)
        if not len(points) or not np.isfinite(points).all():
            raise ValueError('Invalid prior map')
        self.seed = None
        self.request_id = None
        self.session = None
        self.session_subscriptions = []
        self.correction = None
        self.locked = False
        self.ready = False
        self.epoch = uuid.uuid4().hex
        self.wheels = deque(maxlen=1000)
        self.quality = {}
        self.pending = deque(maxlen=20)
        self.receipts = {key: -math.inf for key in ['lidar', 'imu', 'wheel', 'lio', 'match', 'clock']}
        self.stamps = {key: -math.inf for key in ['lidar', 'imu', 'wheel', 'lio', 'match']}
        self.last_clock = 0.
        self.last_pose = None
        self.reason = 'waiting_for_initialpose'
        self.metrics = {}
        self.tf = TransformBroadcaster(self)
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.valid_pub = self.create_publisher(Bool, '/mini_nav/localization_valid', 1)
        self.epoch_pub = self.create_publisher(String, '/mini_nav/localization_epoch', latched)
        self.status_pub = self.create_publisher(String, '/scurm/localization_status', 1)
        self.pose_pub = self.create_publisher(PoseWithCovarianceStamped, '/scurm/localization_pose', 10)
        self.prior_pub = self.create_publisher(PointCloud2, '/scurm/prior_map', latched)
        self.prior_pub.publish(point_cloud2.create_cloud_xyz32(Header(frame_id='map'), points[:, :3]))
        self.cloud_pub = None
        self.request_pub = self.create_publisher(String, '/scurm/localization_request', latched)
        self.odom_pub = self.create_publisher(Odometry, '/scurm/imu_odometry', 10)
        self.match_pub = self.create_publisher(DiagnosticArray, '/scurm/match_quality', 10)
        self.epoch_pub.publish(String(data=self.epoch))
        self.create_subscription(String, '/scurm/backend_session', self.backend_session, latched)
        self.create_subscription(PoseWithCovarianceStamped, '/initialpose', self.manual_initial, 1)
        self.create_subscription(PointCloud2, '/scurm/lidar/points', self.cloud, qos_profile_sensor_data)
        self.create_subscription(Imu, '/imu', lambda msg: self.received('imu', msg), qos_profile_sensor_data)
        self.create_subscription(Odometry, '/scurm/wheel_odometry', self.wheel, 10)
        self.create_timer(.05, self.watchdog, clock=Clock(clock_type=ClockType.STEADY_TIME))

    def received(self, key, msg):
        self.receipts[key] = time.monotonic()
        self.stamps[key] = seconds(msg.header.stamp)

    def initial(self, msg):
        if self.session is None or self.seed is not None or self.locked or msg.header.frame_id != 'map':
            return
        try:
            self.seed = pose_matrix(msg.pose.pose)
            self.get_logger().info('ICP prior aligned; waiting for timestamped FAST-LIO2 match')
        except ValueError as exc:
            self.get_logger().error(str(exc))

    def manual_initial(self, msg):
        if msg.header.frame_id != 'map':
            self.get_logger().warn('Initial pose must use the map frame')
            return
        try:
            matrix = pose_matrix(msg.pose.pose) @ np.linalg.inv(IMU_BASE)
            q = Rotation.from_matrix(matrix[:3, :3]).as_quat()
        except ValueError as exc:
            self.get_logger().error(str(exc))
            return
        self.clear_estimate()
        self.request_id = uuid.uuid4().hex
        self.reason = 'waiting_for_backend_restart'
        request = {'request_id': self.request_id, 'imu_pose': matrix[:3, 3].tolist() + q.tolist()}
        self.request_pub.publish(String(data=json.dumps(request)))
        self.get_logger().info('Accepted /initialpose; navigation revoked, restarting ICP and FAST-LIO2')

    def clear_estimate(self):
        self.session = None
        for subscription in self.session_subscriptions:
            self.destroy_subscription(subscription)
        self.session_subscriptions.clear()
        if self.cloud_pub is not None:
            self.destroy_publisher(self.cloud_pub)
            self.cloud_pub = None
        self.seed = None
        self.correction = None
        self.last_pose = None
        self.quality.clear()
        self.pending.clear()
        self.metrics = {}
        self.locked = False
        self.ready = False
        self.receipts['lio'] = self.receipts['match'] = -math.inf
        self.stamps['lio'] = self.stamps['match'] = -math.inf
        clock = self.get_clock().now().nanoseconds * 1e-9
        if self.wheels and clock < self.wheels[-1][0]:
            self.wheels.clear()
        self.last_clock = clock
        self.epoch = uuid.uuid4().hex
        self.valid_pub.publish(Bool(data=False))
        self.epoch_pub.publish(String(data=self.epoch))

    def backend_session(self, msg):
        try:
            data = json.loads(msg.data)
            uuid.UUID(hex=data['session'])
        except (ValueError, KeyError, TypeError):
            return
        if data.get('request_id') != self.request_id or self.request_id is None or data['session'] == self.session:
            return
        self.clear_estimate()
        self.session = data['session']
        prefix = '/scurm/session_' + self.session
        self.reason = 'waiting_for_icp'
        self.cloud_pub = self.create_publisher(PointCloud2, prefix + '/icp_cloud', 2)
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)

        def guarded(callback):
            session = self.session

            def receive(value):
                if self.session == session:
                    callback(value)
            return receive

        self.session_subscriptions = [
            self.create_subscription(PoseWithCovarianceStamped, prefix + '/icp_result', guarded(self.initial), latched),
            self.create_subscription(Odometry, prefix + '/imu_odometry', guarded(self.odometry), 10),
            self.create_subscription(DiagnosticArray, prefix + '/match_quality', guarded(self.match), 10)]

    def cloud(self, msg):
        self.received('lidar', msg)
        if self.cloud_pub is None or self.seed is not None or self.locked:
            return
        points = point_cloud2.read_points_numpy(msg, field_names=('x', 'y', 'z'), skip_nans=True).reshape(-1, 3)[::2]
        points = points[np.isfinite(points).all(axis=1)]
        points = points[np.linalg.norm(points, axis=1) > .3] + IMU_LIDAR
        header = copy.deepcopy(msg.header)
        header.frame_id = 'imu_link'
        self.cloud_pub.publish(point_cloud2.create_cloud_xyz32(header, points.astype(np.float32)))

    def send_tf(self, parent, child, matrix, stamp):
        msg = TransformStamped()
        msg.header.frame_id, msg.child_frame_id = parent, child
        msg.header.stamp = stamp
        msg.transform.translation.x, msg.transform.translation.y, msg.transform.translation.z = map(float, matrix[:3, 3])
        q = Rotation.from_matrix(matrix[:3, :3]).as_quat()
        msg.transform.rotation.x, msg.transform.rotation.y, msg.transform.rotation.z, msg.transform.rotation.w = map(float, q)
        self.tf.sendTransform(msg)

    def wheel(self, msg):
        try:
            matrix = planar(pose_matrix(msg.pose.pose))
        except ValueError:
            return
        stamp = seconds(msg.header.stamp)
        if self.wheels and stamp <= self.wheels[-1][0]:
            if stamp < self.wheels[-1][0]:
                self.locked = True
                self.reason = 'wheel_timestamp_rewound'
            return
        self.received('wheel', msg)
        self.wheels.append((stamp, matrix))
        self.send_tf('odom', 'base_footprint', matrix, msg.header.stamp)
        if self.correction is not None and not self.locked and time.monotonic() - self.receipts['lio'] <= self.timeout:
            self.send_tf('map', 'odom', self.correction, msg.header.stamp)
        if self.seed is not None:
            self.send_tf('map', 'scurm_lio_odom', self.seed, msg.header.stamp)

    def match(self, msg):
        self.match_pub.publish(msg)
        for status in msg.status:
            if status.name == 'fastlio_prior_match':
                try:
                    self.quality[round(seconds(msg.header.stamp), 6)] = {
                        item.key: float(item.value) for item in status.values}
                    if len(self.quality) > 30:
                        del self.quality[next(iter(self.quality))]
                    self.received('match', msg)
                except ValueError:
                    pass

    def odometry(self, msg):
        if msg.header.frame_id != 'scurm_lio_odom':
            return
        self.odom_pub.publish(msg)
        self.pending.append(msg)

    def process(self):
        while self.pending:
            msg = self.pending[0]
            stamp = seconds(msg.header.stamp)
            metrics = self.quality.get(round(stamp, 6))
            wheel = interpolate(self.wheels, stamp)
            if metrics is None or wheel is None or self.seed is None:
                if seconds(self.get_clock().now().to_msg()) - stamp > self.timeout:
                    self.pending.popleft()
                    continue
                return
            self.pending.popleft()
            self.metrics = metrics
            if not match_valid(metrics, msg.pose.covariance, self.minimum_points,
                               self.minimum_ratio, self.maximum_residual):
                self.reason = 'prior_match_or_covariance_rejected'
                self.receipts['lio'] = -math.inf
                continue
            try:
                map_imu = self.seed @ pose_matrix(msg.pose.pose)
                base = planar(map_imu @ IMU_BASE)
            except ValueError:
                continue
            if self.last_pose is not None:
                old_stamp, old_base = self.last_pose
                distance = np.linalg.norm(base[:2, 3] - old_base[:2, 3])
                angle = Rotation.from_matrix(old_base[:3, :3].T @ base[:3, :3]).magnitude()
                if stamp < old_stamp or distance > .25 + max(0., stamp - old_stamp) * .4 or angle > .3 + max(0., stamp - old_stamp):
                    self.locked = True
                    self.reason = 'estimator_reset_or_jump'
                    return
            self.last_pose = (stamp, base)
            self.correction = base @ np.linalg.inv(wheel)
            self.received('lio', msg)
            pose = PoseWithCovarianceStamped()
            pose.header = copy.deepcopy(msg.header)
            pose.header.frame_id = 'map'
            write_pose(pose.pose.pose, base)
            # FAST-LIO rotation error is a right perturbation in the IMU frame.
            # Include the IMU-to-base lever arm when transporting the xyz/rotation covariance.
            jacobian = np.zeros((6, 6))
            jacobian[:3, :3] = self.seed[:3, :3]
            x, y, z = IMU_BASE[:3, 3]
            skew = np.array([[0., -z, y], [z, 0., -x], [-y, x, 0.]])
            jacobian[:3, 3:] = -map_imu[:3, :3] @ skew
            jacobian[3:, 3:] = map_imu[:3, :3]
            pose.pose.covariance = (jacobian @ np.array(msg.pose.covariance).reshape(6, 6) @ jacobian.T).ravel().tolist()
            self.pose_pub.publish(pose)

    def watchdog(self):
        now = time.monotonic()
        clock = self.get_clock().now().nanoseconds * 1e-9
        if clock < self.last_clock - 1e-6:
            self.locked = True
            self.reason = 'simulation_clock_rewound'
        if clock > self.last_clock:
            self.receipts['clock'] = now
        self.last_clock = clock
        if not self.locked:
            self.process()
        # Processing an estimate records a new monotonic receipt time.
        now = time.monotonic()
        fresh = all(0 <= now - value <= self.timeout for value in self.receipts.values())
        stamped = all(-.05 <= clock - value <= self.timeout for value in self.stamps.values())
        # The latest match may precede an odometry callback; reject a bad fresh match immediately.
        latest = self.quality.get(round(self.stamps['match'], 6), {})
        match_ok = (latest.get('effective_points', 0) >= self.minimum_points and
                    latest.get('matched_ratio', 0) >= self.minimum_ratio and
                    0 <= latest.get('mean_abs_residual', math.inf) <= self.maximum_residual)
        valid = self.seed is not None and self.correction is not None and fresh and stamped and match_ok and not self.locked
        if self.ready and not valid:
            self.epoch = uuid.uuid4().hex
            self.epoch_pub.publish(String(data=self.epoch))
        if valid != self.ready:
            self.get_logger().info(f'Localization valid={valid}: {self.reason if not valid else "prior_match_ok"}')
        self.ready = valid
        reason = 'prior_match_ok' if valid else self.reason if self.locked or self.seed is None else 'waiting_for_fresh_valid_estimate'
        self.valid_pub.publish(Bool(data=valid))
        self.status_pub.publish(String(data=json.dumps({'valid': valid, 'reason': reason, 'epoch': self.epoch,
            'request_id': self.request_id, 'session': self.session,
            'metrics': self.metrics, 'receipt_ages': {key: now - age for key, age in self.receipts.items()},
            'estimate_stamp': self.stamps['lio']})))


def main():
    rclpy.init()
    node = MiniNavLocalizer()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if rclpy.ok():
            node.valid_pub.publish(Bool(data=False))
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
