import importlib.util
from pathlib import Path

import numpy as np
import pytest
from scipy.spatial.transform import Rotation
from geometry_msgs.msg import Pose

spec = importlib.util.spec_from_file_location('geometry', Path(__file__).parents[1] / 'scripts/localization_geometry.py')
geometry = importlib.util.module_from_spec(spec)
spec.loader.exec_module(geometry)


def matrix(x, y, yaw):
    out = np.eye(4)
    out[:3, :3] = Rotation.from_euler('z', yaw).as_matrix()
    out[:2, 3] = [x, y]
    return out


def test_wheel_interpolation_wraps_yaw_at_same_measurement_time():
    value = geometry.interpolate([(1., matrix(0, 0, np.deg2rad(179))),
                                  (1.1, matrix(1, 2, np.deg2rad(-179)))], 1.05)
    np.testing.assert_allclose(value[:2, 3], [.5, 1.])
    assert abs(abs(np.arctan2(value[1, 0], value[0, 0])) - np.pi) < 1e-10
    assert geometry.interpolate([(1., matrix(0, 0, 0)), (1.3, matrix(1, 0, 0))], 1.1) is None
    assert geometry.interpolate([(1., matrix(0, 0, 0)), (1.1, matrix(1, 0, 0))], .9) is None


def test_dynamic_correction_composes_with_arbitrary_wheel_origin():
    seed, lio, wheel = matrix(.2, -.1, .1), matrix(1., .3, .4), matrix(-2., 3., -.3)
    base = geometry.planar(seed @ lio @ geometry.IMU_BASE)
    correction = base @ np.linalg.inv(wheel)
    np.testing.assert_allclose(correction @ wheel, base, atol=1e-12)
    assert base[2, 3] == 0
    assert np.linalg.norm(base[:2, 3] - (seed @ lio)[:2, 3]) == pytest.approx(.032)


@pytest.mark.parametrize('points,ratio,residual', [(59, .9, .02), (100, .1, .02), (100, .9, .081),
                                                (100, .9, float('nan')), (100, .9, -.1)])
def test_rejects_bad_match(points, ratio, residual):
    metrics = dict(effective_points=points, matched_ratio=ratio, mean_abs_residual=residual)
    assert not geometry.match_valid(metrics, np.eye(6).ravel() * .01, 60, .2, .08)


def test_rejects_covariance_and_invalid_pose():
    metrics = dict(effective_points=100, matched_ratio=.9, mean_abs_residual=.02)
    assert geometry.match_valid(metrics, np.eye(6).ravel() * .01, 60, .2, .08)
    cov = np.eye(6) * .01
    cov[0, 0] = .3
    assert not geometry.match_valid(metrics, cov.ravel(), 60, .2, .08)
    pose = Pose()
    pose.orientation.w = 0.
    with pytest.raises(ValueError):
        geometry.pose_matrix(pose)
