#!/usr/bin/env python3
"""Real-node acceptance for manual initial pose and in-motion reinitialization."""
import argparse
import json
import math
from pathlib import Path
import time

import numpy as np
import rclpy
from rclpy.qos import QoSProfile, DurabilityPolicy
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav2_msgs.action import NavigateToPose
from std_msgs.msg import String

from validate_localization_navigation import Probe


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    rclpy.init()
    node = Probe()
    latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
    subscription = node.create_subscription(String, '/scurm/backend_status',
        lambda msg: node.record('backend', json.loads(msg.data)), latched)
    initial_pub = node.create_publisher(PoseWithCovarianceStamped, '/initialpose', 10)
    result = {'passed': False}

    def observe(seconds):
        end = time.monotonic() + seconds
        node.until(lambda: time.monotonic() >= end, seconds + .2)

    def set_pose(x, y, yaw, frame='map', quaternion_valid=True):
        msg = PoseWithCovarianceStamped()
        msg.header.frame_id = frame
        msg.header.stamp = node.get_clock().now().to_msg()
        msg.pose.pose.position.x, msg.pose.pose.position.y = float(x), float(y)
        msg.pose.pose.orientation.z = math.sin(yaw / 2) if quaternion_valid else 0.
        msg.pose.pose.orientation.w = math.cos(yaw / 2) if quaternion_valid else 0.
        msg.pose.covariance[0] = msg.pose.covariance[7] = .25
        msg.pose.covariance[35] = .0685
        initial_pub.publish(msg)
        node.event('initialpose_sent', pose=[x, y, yaw], frame=frame, quaternion_valid=quaternion_valid)

    def localized():
        backend = node.latest.get('backend', {})
        localizer = node.latest.get('localizer', {})
        return (node.latest.get('valid') and node.latest.get('local_valid') and
                localizer.get('session') == backend.get('session') and backend.get('session') is not None)

    try:
        if not node.until(lambda: 'backend' in node.latest and 'wheel' in node.latest and
                          'command' in node.latest and initial_pub.get_subscription_count() >= 2, 15.):
            raise RuntimeError('Initial-pose subscribers or simulation not ready')
        observe(1.)
        if node.latest.get('valid') or node.latest['backend']['state'] != 'waiting_for_initialpose':
            raise RuntimeError('Localization started without an initial pose')
        if max(map(abs, node.latest['command'])) > 1e-8:
            raise RuntimeError('Robot moved before initialization')
        if not node.action.wait_for_server(timeout_sec=3.):
            raise RuntimeError('Navigation action unavailable')
        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = 'map'
        goal.pose.pose.position.y = 1.
        goal.pose.pose.orientation.w = 1.
        handle = node.future(node.action.send_goal_async(goal), 3.)
        if handle.accepted:
            raise RuntimeError('Unlocalized robot accepted navigation')
        result['before_initialpose'] = {'valid': False, 'navigation_rejected': True, 'command': node.latest['command']}
        node.event('waiting_for_initialpose_verified')
        set_pose(0., 0., 0., frame='odom')
        observe(.3)
        set_pose(0., 0., 0., quaternion_valid=False)
        observe(.3)
        if node.latest['backend']['state'] != 'waiting_for_initialpose':
            raise RuntimeError('Invalid initial pose started a backend')
        result['invalid_initialposes_rejected'] = True
        set_pose(.07, -.04, .12)
        if not node.until(localized, 25.):
            raise RuntimeError('Manual initial pose did not initialize localization')
        first = dict(node.latest['backend'])
        result['first_localization'] = {'session': first['session'], 'frontend_pid': first['pids']['frontend'],
                                        'pose': node.latest['pose'], 'metrics': node.latest['localizer']['metrics']}
        node.event('first_pose_localized', **result['first_localization'])
        handle = node.send(0., 1., math.pi / 2)
        interrupted = handle.get_result_async()
        start_wheel = np.array(node.latest['wheel'][:2])
        if not node.until(lambda: abs(node.latest.get('command', [0, 0])[0]) > .02 and
                          np.linalg.norm(np.array(node.latest['wheel'][:2]) - start_wheel) > .04, 15.):
            raise RuntimeError('Robot did not move before reinitialization')
        old_epoch = node.latest['localizer']['epoch']
        current = node.latest['pose']
        reset_at = time.monotonic()
        set_pose(current[0] + .03, current[1] - .02, current[2] + .08)
        if not node.until(lambda: node.latest.get('valid') is False and
                          node.latest['localizer']['epoch'] != old_epoch and
                          max(map(abs, node.latest['command'])) < 1e-8, 2.):
            raise RuntimeError('Reinitialization did not revoke motion')
        result['reinitialization_stop_wall_seconds'] = time.monotonic() - reset_at
        wrapped = node.future(interrupted, 3.)
        if wrapped.status != 6:
            raise RuntimeError('Old navigation task was not aborted')
        result['old_task'] = {'status': wrapped.status, 'error_msg': wrapped.result.error_msg}
        if not node.until(lambda: localized() and node.latest['backend']['session'] != first['session'], 25.):
            raise RuntimeError('Repeated initial pose did not relocalize')
        second = node.latest['backend']
        if second['pids']['frontend'] == first['pids']['frontend'] or Path(f"/proc/{first['pids']['frontend']}").exists():
            raise RuntimeError('Old FAST-LIO2 process survived reinitialization')
        result['second_localization'] = {'session': second['session'], 'frontend_pid': second['pids']['frontend'],
                                         'pose': node.latest['pose'], 'metrics': node.latest['localizer']['metrics']}
        observe(1.)
        if max(map(abs, node.latest['command'])) > 1e-8:
            raise RuntimeError('Old goal resumed without a new goal')
        node.event('reinitialized_and_old_task_stopped', delay=result['reinitialization_stop_wall_seconds'])
        handle = node.send(0., 1., math.pi / 2)
        wrapped = node.future(handle.get_result_async(), 60.)
        if wrapped.status != 4:
            raise RuntimeError(f'Navigation after reinitialization failed: {wrapped.result.error_msg}')
        observe(.3)
        result['navigation_after_reinitialization'] = {'status': wrapped.status,
            'error_msg': wrapped.result.error_msg, 'settled_pose': node.latest['pose'], 'command': node.latest['command']}
        if max(map(abs, node.latest['command'])) > 1e-8:
            raise RuntimeError('Completed navigation did not stop')
        publishers = [p.node_name for p in node.get_publishers_info_by_topic('/cmd_vel')]
        if publishers != ['velocity_guard']:
            raise RuntimeError('Velocity publisher ownership changed')
        result['cmd_vel_publishers'] = publishers
        result['passed'] = True
        node.event('acceptance_passed', **result['navigation_after_reinitialization'])
    except Exception as exc:
        result['error'] = str(exc)
        node.event('failure', error=str(exc), latest=node.latest)
        if node.active is not None:
            try:
                node.future(node.active.cancel_goal_async(), 2.)
            except Exception:
                pass
    finally:
        node.sample()
        result['events'] = node.events
        result['samples'] = node.samples
        result['final'] = node.latest
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(json.dumps(result, indent=2) + '\n')
        node.destroy_node()
        rclpy.shutdown()
    raise SystemExit(0 if result['passed'] else 1)


if __name__ == '__main__':
    main()
