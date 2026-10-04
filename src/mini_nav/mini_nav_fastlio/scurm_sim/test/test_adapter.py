import importlib.util
from pathlib import Path

import numpy as np
import pytest
import rclpy
from geometry_msgs.msg import Pose
from nav_msgs.msg import Odometry
from scipy.spatial.transform import Rotation

spec = importlib.util.spec_from_file_location('scurm_adapter', Path(__file__).parents[3] / 'mini_nav_nodes/src/localization/fastlio2/adapter/adapter.py')
adapter = importlib.util.module_from_spec(spec)
spec.loader.exec_module(adapter)


def test_rotated_imu_to_base():
    pose = Pose()
    pose.orientation.z = np.sin(np.pi / 4)
    pose.orientation.w = np.cos(np.pi / 4)
    pose.position.x = 1.0
    body = adapter.pose_matrix(pose) @ adapter.IMU_BASE
    np.testing.assert_allclose(body[:3, 3], [1.0, 0.032, -0.078], atol=1e-12)
    recovered = Pose()
    adapter.write_pose(recovered, body)
    np.testing.assert_allclose(adapter.pose_matrix(recovered), body, atol=1e-12)


def test_invalid_pose_rejected():
    invalid = Pose()
    invalid.orientation.w = 0.0
    with pytest.raises(ValueError):
        adapter.pose_matrix(invalid)
    pose = Pose()
    pose.orientation.w = 1.0
    pose.position.x = float('nan')
    with pytest.raises(ValueError):
        adapter.pose_matrix(pose)


@pytest.mark.parametrize('args', [
    (False, 0, 0, 0, 0), (True, 0.6, 0, 0, 0), (True, 0, 0.6, 0, 0),
    (True, 0, -0.1, 0, 0), (True, 0, 0, 0.4, 0), (True, 0, 0, 0, 0.6)])
def test_watchdog_blocks_stale_or_uninitialized_input(args):
    assert not adapter.command_allowed(*args)


def test_watchdog_accepts_fresh_inputs():
    assert adapter.command_allowed(True, 0.1, 0.1, 0.1, 0.1)


def test_mapping_odometry_has_one_dynamic_tf_without_map_frame():
    rclpy.init(args=['--ros-args', '-p', 'mapping_mode:=true'])
    node = adapter.Adapter()
    transforms = []

    class Recorder:
        def sendTransform(self, values):
            transforms.extend(values)

    node.tf = Recorder()
    try:
        msg = Odometry()
        msg.header.stamp.sec = 1
        msg.pose.pose.position.x = 1.0
        msg.pose.pose.orientation.z = np.sin(np.pi / 4)
        msg.pose.pose.orientation.w = np.cos(np.pi / 4)
        node.odometry(msg)
        assert [(tf.header.frame_id, tf.child_frame_id) for tf in transforms] == [
            ('odom', 'base_footprint')]
        translation = transforms[0].transform.translation
        np.testing.assert_allclose([translation.x, translation.y, translation.z],
                                   [1.0, 0.032, -0.078], atol=1e-12)
        assert node.odom_stamp == 1.0
    finally:
        node.destroy_node()
        rclpy.shutdown()
