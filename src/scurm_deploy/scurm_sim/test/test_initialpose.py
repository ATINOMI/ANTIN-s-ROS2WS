import json
from pathlib import Path
import sys
import uuid

import numpy as np
import pytest
import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped
from std_msgs.msg import String

sys.path.insert(0, str(Path(__file__).parents[1] / 'scripts'))
from mini_nav_localizer import MiniNavLocalizer
from localization_backend import parse_request


@pytest.fixture
def localizer():
    prior = Path(__file__).parents[1] / 'maps/local_prior.pcd'
    rclpy.init(args=['--ros-args', '-p', f'prior_map_path:={prior}'], domain_id=224)
    node = MiniNavLocalizer()
    yield node
    node.destroy_node()
    rclpy.shutdown()


def pose(frame='map'):
    msg = PoseWithCovarianceStamped()
    msg.header.frame_id = frame
    msg.pose.pose.orientation.w = 1.
    return msg


def test_waits_for_initialpose_and_ignores_unsolicited_seed(localizer):
    assert localizer.request_id is None
    assert localizer.cloud_pub is None
    assert not localizer.ready
    localizer.initial(pose())
    assert localizer.seed is None


def test_valid_pose_revokes_state_and_converts_base_to_imu(localizer, monkeypatch):
    localizer.ready = True
    localizer.seed = np.eye(4)
    localizer.correction = np.eye(4)
    epoch = localizer.epoch
    localizer.manual_initial(pose('odom'))
    assert localizer.ready and localizer.epoch == epoch
    bad = pose()
    bad.pose.pose.orientation.w = 0.
    localizer.manual_initial(bad)
    assert localizer.ready and localizer.epoch == epoch
    captured = []
    monkeypatch.setattr(localizer.request_pub, 'publish', lambda m: captured.append(m.data))
    msg = pose()
    msg.pose.pose.position.x, msg.pose.pose.position.y = 1., 2.
    msg.pose.pose.orientation.w = msg.pose.pose.orientation.z = np.sqrt(.5)
    localizer.manual_initial(msg)
    assert not localizer.ready and localizer.seed is None and localizer.correction is None
    assert localizer.epoch != epoch
    request_id, imu_pose = parse_request(captured[0])
    assert request_id == localizer.request_id
    np.testing.assert_allclose(imu_pose[:3], [1., 1.968, .078], atol=1e-12)


def test_old_icp_session_cannot_seed_new_request(localizer):
    localizer.manual_initial(pose())
    first_request = localizer.request_id
    first_session = uuid.uuid4().hex
    localizer.backend_session(String(data=json.dumps({'request_id': first_request, 'session': first_session})))
    old_seed_callback = localizer.session_subscriptions[0].callback
    old_seed_callback(pose())
    assert localizer.seed is not None
    localizer.manual_initial(pose())
    assert localizer.seed is None
    old_seed_callback(pose())
    assert localizer.seed is None
    localizer.backend_session(String(data=json.dumps({'request_id': first_request, 'session': uuid.uuid4().hex})))
    assert localizer.session is None
    localizer.backend_session(String(data=json.dumps({'request_id': localizer.request_id, 'session': uuid.uuid4().hex})))
    assert len(localizer.session_subscriptions) == 3
    localizer.session_subscriptions[0].callback(pose())
    assert localizer.seed is not None


@pytest.mark.parametrize('value', [[0., 0., 0., 0., 0., 0., 0.],
                                  [0., 0., 0., 0., 0., 0., float('nan')], [0., 1.]])
def test_backend_rejects_invalid_initial_pose(value):
    with pytest.raises(ValueError):
        parse_request(json.dumps({'request_id': uuid.uuid4().hex, 'imu_pose': value}))
