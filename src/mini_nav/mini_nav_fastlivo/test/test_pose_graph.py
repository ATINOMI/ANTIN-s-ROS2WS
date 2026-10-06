"""验证回访约束、固定首帧、历史修正及不具备回环时明确失败。"""
import numpy as np
import pytest
from scipy.spatial.transform import Rotation
from mini_nav_fastlivo.geometry import transform
from mini_nav_fastlivo.pose_graph import correct_trajectory, match_loop, relative, optimize_graph


def test_real_scan_revisit_corrects_drift_without_changing_record():
    rng = np.random.default_rng(13)
    world = rng.uniform(-2, 2, (5000, 3))
    world[:1700, 0] = -2
    world[1700:3400, 1] = 2
    world[3400:, 2] = -1
    truth = np.repeat(np.eye(4)[None], 5, axis=0)
    truth[:, :2, 3] = [[0, 0], [1, 0], [1, 1], [0, 1], [0, 0]]
    poses = truth.copy()
    poses[:, 0, 3] += np.arange(5)*.03
    original = poses.copy()
    clouds = [transform(world, np.linalg.inv(p)) for p in truth]
    selected, corrected, report = correct_trajectory(np.arange(5)*20_000_000_000, poses, clouds)
    assert len(report['loops']) >= 1
    assert np.linalg.norm(corrected[-1, :2, 3]) < .035
    assert np.allclose(corrected[0], poses[0])
    assert np.array_equal(poses, original)
    assert selected[0] == 0 and selected[-1] == 4


def test_stationary_observations_do_not_invent_a_loop():
    poses = np.repeat(np.eye(4)[None], 20, axis=0)
    clouds = [np.random.default_rng(1).normal(size=(300, 3))]*20
    with pytest.raises(ValueError, match='No verified'):
        correct_trajectory(np.arange(20)*2_000_000_000, poses, clouds)


def test_unrelated_cloud_is_rejected():
    points = np.random.default_rng(11).normal(size=(2000, 3))
    assert match_loop(points, points+[30, 30, 0], np.eye(4)) is None


def test_graph_handles_angle_wrap_and_nonzero_anchor():
    initial = np.array([[4., -2., 3.12], [3., -2., -3.12], [4.12, -2., 3.12]])
    truth = initial.copy()
    truth[-1] = truth[0]
    edges = [dict(i=i-1, j=i, measurement=relative(initial[i-1], initial[i]),
                  sigma=[.03, .03, .02]) for i in range(1, 3)]
    edges.append(dict(i=0, j=2, measurement=relative(truth[0], truth[2]), sigma=[.01]*3))
    result, report = optimize_graph(initial, edges)
    assert np.array_equal(result[0], initial[0])
    assert np.linalg.norm(result[-1, :2]-truth[-1, :2]) < .02
    assert report['cost_after'] < report['cost_before']
