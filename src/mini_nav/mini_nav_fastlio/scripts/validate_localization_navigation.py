#!/usr/bin/env python3
"""Exercise the real mini_nav Actions and FAST-LIO2-loss guard in Gazebo."""
import argparse
import json
import math
import os
from pathlib import Path
import signal
import time

import numpy as np
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy
from geometry_msgs.msg import PoseWithCovarianceStamped, TwistStamped
from nav_msgs.msg import Odometry, Path as NavPath
from nav2_msgs.action import NavigateToPose
from std_msgs.msg import Bool, String
from tf2_msgs.msg import TFMessage


class Probe(Node):
    def __init__(self):
        super().__init__('scurm_navigation_acceptance', parameter_overrides=[
            rclpy.parameter.Parameter('use_sim_time', value=True)])
        self.started = time.monotonic()
        self.latest = {}
        self.samples = []
        self.events = []
        self.tf_counts = {}
        self.action = ActionClient(self, NavigateToPose, '/navigate_to_pose')
        self.active = None
        self.create_subscription(PoseWithCovarianceStamped, '/scurm/localization_pose', self.pose, 10)
        self.create_subscription(Odometry, '/scurm/wheel_odometry', self.wheel, 10)
        self.create_subscription(String, '/scurm/localization_status', lambda m: self.record('localizer', json.loads(m.data)), 10)
        self.create_subscription(String, '/mini_nav/navigation_status', lambda m: self.record('task_status', m.data), 10)
        self.create_subscription(Bool, '/mini_nav/localization_valid', lambda m: self.record('valid', m.data), 1)
        self.create_subscription(Bool, '/mini_nav/local_costmap_valid', lambda m: self.record('local_valid', m.data), 1)
        self.create_subscription(TwistStamped, '/cmd_vel', self.command, 10)
        self.create_subscription(NavPath, '/mini_nav/global_path', lambda m: self.record('path_points', len(m.poses)), 10)
        self.create_subscription(TFMessage, '/tf', self.transforms, 100)
        self.create_timer(.1, self.sample)

    def record(self, key, value):
        self.latest[key] = value

    def pose(self, msg):
        p, q = msg.pose.pose.position, msg.pose.pose.orientation
        self.latest['pose'] = [p.x, p.y, math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y*q.y + q.z*q.z))]

    def wheel(self, msg):
        p, q = msg.pose.pose.position, msg.pose.pose.orientation
        self.latest['wheel'] = [p.x, p.y, math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y*q.y + q.z*q.z))]

    def command(self, msg):
        self.latest['command'] = [msg.twist.linear.x, msg.twist.angular.z]

    def transforms(self, msg):
        for tf in msg.transforms:
            key = tf.header.frame_id + '->' + tf.child_frame_id
            self.tf_counts[key] = self.tf_counts.get(key, 0) + 1

    def sample(self):
        self.samples.append({'elapsed': time.monotonic() - self.started, **self.latest})

    def event(self, name, **fields):
        item = {'elapsed': time.monotonic() - self.started, 'name': name, **fields}
        self.events.append(item)
        print(json.dumps(item), flush=True)

    def until(self, predicate, timeout):
        end = time.monotonic() + timeout
        while time.monotonic() < end:
            rclpy.spin_once(self, timeout_sec=.05)
            if predicate():
                return True
        return False

    def future(self, future, timeout):
        if not self.until(future.done, timeout):
            raise RuntimeError('Action future timed out')
        return future.result()

    def send(self, x, y, yaw=0.):
        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = 'map'
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x, goal.pose.pose.position.y = float(x), float(y)
        goal.pose.pose.orientation.z, goal.pose.pose.orientation.w = math.sin(yaw / 2), math.cos(yaw / 2)
        handle = self.future(self.action.send_goal_async(goal), 5.)
        self.event('goal_sent', goal=[x, y, yaw], accepted=handle.accepted)
        if not handle.accepted:
            raise RuntimeError('Navigation goal rejected')
        self.active = handle
        return handle


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--frontend-pid', type=int, required=True)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--goal', type=float, nargs=3, default=[0., 1., math.pi / 2])
    args = parser.parse_args()
    proc = Path(f'/proc/{args.frontend_pid}')
    if b'scurm_fastlio_localization' not in (proc / 'cmdline').read_bytes() or b'ROS_DOMAIN_ID=227\0' not in (proc / 'environ').read_bytes():
        raise RuntimeError('Refusing to signal a different frontend')
    rclpy.init()
    node = Probe()
    suspended = False
    result = {'passed': False}
    try:
        if not node.until(lambda: node.latest.get('valid') and node.latest.get('local_valid') and 'pose' in node.latest, 30.):
            raise RuntimeError('Localization or local costmap not ready')
        if not node.action.wait_for_server(timeout_sec=5.):
            raise RuntimeError('NavigateToPose server unavailable')
        publishers = node.get_publishers_info_by_topic('/cmd_vel')
        result['cmd_vel_publishers'] = [p.node_name for p in publishers]
        if result['cmd_vel_publishers'] != ['velocity_guard']:
            raise RuntimeError('Velocity publisher ownership is invalid')
        result['initial'] = dict(node.latest)
        node.event('ready', pose=node.latest['pose'])
        handle = node.send(*args.goal)
        wrapped = node.future(handle.get_result_async(), 90.)
        result['outbound'] = {'status': wrapped.status, 'error_code': wrapped.result.error_code,
                              'error_msg': wrapped.result.error_msg, 'final_pose': node.latest.get('pose')}
        result['outbound']['requested_position_error_m'] = math.hypot(
            node.latest['pose'][0] - args.goal[0], node.latest['pose'][1] - args.goal[1])
        node.event('outbound_result', **result['outbound'])
        if wrapped.status != 4 or result['outbound']['requested_position_error_m'] > .15:
            raise RuntimeError('Outbound navigation did not succeed')
        node.until(lambda: max(map(abs, node.latest.get('command', [1, 1]))) < 1e-8, 1.)
        wheel_at_fault_goal = node.latest['wheel'][:2]
        fault_handle = node.send(0., 0., 0.)
        fault_result = fault_handle.get_result_async()
        if not node.until(lambda: abs(node.latest.get('command', [0, 0])[0]) > .02 and
                          np.linalg.norm(np.array(node.latest['wheel'][:2]) - wheel_at_fault_goal) > .03, 20.):
            raise RuntimeError('Robot did not translate before fault injection')
        old_epoch = node.latest['localizer']['epoch']
        fault_at = time.monotonic()
        os.kill(args.frontend_pid, signal.SIGSTOP)
        suspended = True
        node.event('frontend_suspended', pid=args.frontend_pid)
        if not node.until(lambda: node.latest.get('valid') is False and
                          node.latest['localizer']['epoch'] != old_epoch and
                          max(map(abs, node.latest.get('command', [1, 1]))) < 1e-8, 3.):
            raise RuntimeError('Invalid localization did not revoke motion')
        result['fault_stop_wall_seconds'] = time.monotonic() - fault_at
        wrapped = node.future(fault_result, 5.)
        result['fault_task'] = {'status': wrapped.status, 'error_code': wrapped.result.error_code,
                                'error_msg': wrapped.result.error_msg}
        if wrapped.status == 4:
            raise RuntimeError('Interrupted task must not succeed')
        node.event('fault_stopped', delay=result['fault_stop_wall_seconds'], **result['fault_task'])
        os.kill(args.frontend_pid, signal.SIGCONT)
        suspended = False
        if not node.until(lambda: node.latest.get('valid') is True, 15.):
            raise RuntimeError('Localization did not recover')
        end = time.monotonic() + 1.
        while time.monotonic() < end:
            rclpy.spin_once(node, timeout_sec=.05)
            if max(map(abs, node.latest.get('command', [1, 1]))) > 1e-8:
                raise RuntimeError('Old task resumed without a new goal')
        node.event('recovered_requires_new_goal')
        handle = node.send(0., 0., 0.)
        wrapped = node.future(handle.get_result_async(), 90.)
        result['return'] = {'status': wrapped.status, 'error_code': wrapped.result.error_code,
                            'error_msg': wrapped.result.error_msg, 'final_pose': node.latest.get('pose')}
        node.event('return_result', **result['return'])
        if wrapped.status != 4:
            raise RuntimeError('Return navigation did not succeed')
        pose = node.latest['pose']
        result['return_position_error_m'] = math.hypot(pose[0], pose[1])
        result['return_yaw_error_rad'] = abs(pose[2])
        if not node.until(lambda: max(map(abs, node.latest.get('command', [1, 1]))) < 1e-8, 1.):
            raise RuntimeError('Completed task did not stop')
        result['passed'] = True
    except Exception as exc:
        result['error'] = str(exc)
        node.event('failure', error=str(exc), latest=node.latest)
    finally:
        if suspended:
            os.kill(args.frontend_pid, signal.SIGCONT)
        if not result['passed'] and node.active is not None:
            try:
                node.future(node.active.cancel_goal_async(), 2.)
            except Exception:
                pass
        result['events'] = node.events
        result['samples'] = node.samples
        result['tf_message_counts'] = node.tf_counts
        result['final'] = node.latest
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(json.dumps(result, indent=2) + '\n')
        node.destroy_node()
        rclpy.shutdown()
    raise SystemExit(0 if result['passed'] else 1)


if __name__ == '__main__':
    main()
