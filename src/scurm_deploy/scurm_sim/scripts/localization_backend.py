#!/usr/bin/env python3
"""Restart SCURM's one-shot ICP and fixed-prior filter for each initial pose."""
import glob
import json
import math
import os
from pathlib import Path
import signal
import subprocess
import tempfile
import time
import uuid

from ament_index_python.packages import get_package_prefix
import numpy as np
from scipy.spatial.transform import Rotation
import yaml
import rclpy
from rclpy.clock import Clock, ClockType
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy
from std_msgs.msg import String


def parse_request(data):
    request = json.loads(data)
    uuid.UUID(hex=request['request_id'])
    pose = np.asarray(request['imu_pose'], dtype=float)
    if pose.shape != (7,) or not np.isfinite(pose).all() or abs(np.linalg.norm(pose[3:]) - 1.) > .05:
        raise ValueError('Invalid IMU initial pose')
    return request['request_id'], pose


class LocalizationBackend(Node):
    def __init__(self):
        super().__init__('scurm_localization_backend')
        self.prior = self.declare_parameter('prior_map_path', '').value
        self.config = self.declare_parameter('frontend_config', '').value
        if not Path(self.prior).is_file() or not Path(self.config).is_file():
            raise ValueError('Missing FAST-LIO2 configuration or prior map')
        self.frontend = str(Path(get_package_prefix('fast_lio')) / 'lib/fast_lio/fastlio_mapping')
        self.icp = str(Path(get_package_prefix('icp_relocalization')) / 'lib/icp_relocalization/icp_node')
        self.env = os.environ.copy()
        usb = glob.glob('/usr/lib/*-linux-gnu/libusb-1.0.so.0')
        if usb:
            self.env['LD_PRELOAD'] = usb[0]
        self.temp = tempfile.TemporaryDirectory(prefix='scurm_localization_')
        self.children = {}
        self.pending = None
        self.request_id = None
        self.session = None
        self.stopping_since = None
        self.stop_stage = 0
        self.state = 'waiting_for_initialpose'
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.session_pub = self.create_publisher(String, '/scurm/backend_session', latched)
        self.status_pub = self.create_publisher(String, '/scurm/backend_status', latched)
        self.create_subscription(String, '/scurm/localization_request', self.request, latched)
        self.create_timer(.05, self.tick, clock=Clock(clock_type=ClockType.STEADY_TIME))
        self.publish_status()

    def publish_status(self):
        self.status_pub.publish(String(data=json.dumps({'state': self.state, 'request_id': self.request_id,
            'session': self.session, 'pids': {name: p.pid for name, p in self.children.items() if p.poll() is None}})))

    def request(self, msg):
        try:
            request_id, pose = parse_request(msg.data)
        except (ValueError, KeyError, TypeError) as exc:
            self.get_logger().warn(f'Rejected localization request: {exc}')
            return
        if request_id == self.request_id:
            return
        self.request_id = request_id
        self.pending = (request_id, pose)
        self.state = 'restarting_for_initialpose'
        if self.stopping_since is None:
            self.stopping_since = time.monotonic()
            self.stop_stage = 0
            self.signal_children(signal.SIGINT)
        self.publish_status()

    def signal_children(self, sig):
        for child in self.children.values():
            if child.poll() is None:
                try:
                    child.send_signal(sig)
                except ProcessLookupError:
                    pass

    def parameters(self, name, values):
        path = Path(self.temp.name) / f'{name}.yaml'
        path.write_text(yaml.safe_dump({'/**': {'ros__parameters': values}}, sort_keys=False))
        return str(path)

    def start(self, request_id, pose):
        self.session = uuid.uuid4().hex
        prefix = '/scurm/session_' + self.session
        self.session_pub.publish(String(data=json.dumps({'request_id': request_id, 'session': self.session})))
        yaw = float(math.atan2(Rotation.from_quat(pose[3:]).as_matrix()[1, 0],
                               Rotation.from_quat(pose[3:]).as_matrix()[0, 0]))
        icp_params = self.parameters('icp', {'use_sim_time': True,
            'initial_x': float(pose[0]), 'initial_y': float(pose[1]), 'initial_z': float(pose[2]), 'initial_a': yaw,
            'map_path': self.prior, 'pcl_type': 'standard', 'rotate_input_x_180': False,
            'max_correspondence_distance': .4, 'fitness_score_thre': .005,
            'map_voxel_leaf_size': .08, 'cloud_voxel_leaf_size': .08, 'converged_count_thre': 5})
        frontend_params = self.parameters('frontend', {'use_sim_time': True, 'locate_in_prior_map': True,
            'prior_map_path': self.prior, 'publish_tf': False, 'common.odom_frame_id': 'scurm_lio_odom',
            'publish.ikd_tree_en': False, 'publish.map_en': False})
        # Each restart has private result topics; old transient ICP results cannot seed a new filter.
        frontend_args = [self.frontend, '--ros-args', '-r', '__node:=scurm_fastlio_localization',
            '--params-file', self.config, '--params-file', frontend_params,
            '-r', '/icp_result:=' + prefix + '/icp_result',
            '-r', '/Odometry:=' + prefix + '/imu_odometry',
            '-r', '/scurm/match_quality:=' + prefix + '/match_quality']
        icp_args = [self.icp, '--ros-args', '-r', '__node:=scurm_icp_initializer', '--params-file', icp_params,
            '-r', '/pointcloud2:=' + prefix + '/icp_cloud',
            '-r', '/icp_result:=' + prefix + '/icp_result']
        self.children['frontend'] = subprocess.Popen(frontend_args, env=self.env, start_new_session=True)
        self.children['icp'] = subprocess.Popen(icp_args, env=self.env, start_new_session=True)
        self.state = 'running'
        self.get_logger().info(f'Started localization session {self.session} from /initialpose')
        self.publish_status()

    def tick(self):
        if self.pending is not None:
            if any(p.poll() is None for p in self.children.values()):
                age = time.monotonic() - self.stopping_since
                if age > 2. and self.stop_stage == 0:
                    self.signal_children(signal.SIGTERM)
                    self.stop_stage = 1
                elif age > 3. and self.stop_stage == 1:
                    self.signal_children(signal.SIGKILL)
                    self.stop_stage = 2
                return
            self.children.clear()
            request = self.pending
            self.pending = None
            self.stopping_since = None
            try:
                self.start(*request)
            except OSError as exc:
                self.signal_children(signal.SIGTERM)
                self.state = 'backend_start_failed'
                self.get_logger().error(str(exc))
                self.publish_status()
        elif self.children and self.state == 'running':
            frontend_failed = self.children['frontend'].poll() is not None
            icp_code = self.children['icp'].poll()
            if frontend_failed or (icp_code is not None and icp_code != 0):
                self.state = 'backend_failed_requires_initialpose'
                self.signal_children(signal.SIGTERM)
                self.publish_status()

    def close(self):
        self.signal_children(signal.SIGINT)
        for child in self.children.values():
            try:
                child.wait(timeout=1.)
            except subprocess.TimeoutExpired:
                child.terminate()
                try:
                    child.wait(timeout=1.)
                except subprocess.TimeoutExpired:
                    child.kill()
                    child.wait()
        self.temp.cleanup()


def main():
    rclpy.init()
    node = LocalizationBackend()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.close()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
