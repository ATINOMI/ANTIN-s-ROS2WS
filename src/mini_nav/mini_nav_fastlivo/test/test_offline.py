"""验证原始记录完整性与 SE3 校正插值，避免优化输入错位。"""
import json
import numpy as np
import pytest
from scipy.spatial.transform import Rotation
from mini_nav_fastlivo.session import SessionWriter
from mini_nav_fastlivo.offline import read_session, interpolate_corrections, matrices_from_rows, pose_rows
from mini_nav_fastlivo.bundle import load_bundle, save_bundle
from mini_nav_fastlivo.grid import HeightGrid, GeometryStore


def metadata():
    value = {name: np.eye(4).tolist() for name in ['map_world', 'imu_lidar', 'imu_base']}
    value.update(resolution=.05, map_size=4., min_height=.02, max_height=.4)
    return value


def test_recording_index_hash_and_clock_reset(tmp_path):
    writer = SessionWriter(tmp_path, metadata())
    points = np.random.default_rng(2).normal(size=(100, 3))
    for stamp in range(15):
        writer.append(stamp, points, np.eye(4))
    with pytest.raises(ValueError, match='increase'):
        writer.append(14, points, np.eye(4))
    writer.close()
    saved, stamps, poses, clouds = read_session(writer.path)
    assert saved['state'] == 'complete' and len(stamps) == 15
    assert np.allclose(clouds[0], points) and np.allclose(poses[0], np.eye(4))
    (writer.path / '000003.npz').write_bytes(b'corrupt')
    with pytest.raises(ValueError, match='hash'):
        read_session(writer.path)


def test_pose_order_and_nonzero_initial_yaw():
    poses = np.repeat(np.eye(4)[None], 3, axis=0)
    poses[:, :3, :3] = Rotation.from_euler('z', [.6, .7, .9]).as_matrix()
    poses[:, :3, 3] = [[1, 2, 3], [2, 3, 4], [3, 4, 5]]
    assert np.allclose(matrices_from_rows(pose_rows(poses)), poses)


def test_interpolate_correction_preserves_raw_input():
    raw = np.repeat(np.eye(4)[None], 5, axis=0)
    original = raw.copy()
    raw[:, 0, 3] = np.arange(5)
    original[:] = raw
    selected = np.array([0, 4])
    optimized = raw[selected].copy()
    optimized[1, 1, 3] = .08
    optimized[1, :3, :3] = Rotation.from_euler('z', .02).as_matrix()
    result = interpolate_corrections(np.arange(5)*100000000, raw, selected, optimized)
    assert np.allclose(raw, original)
    assert np.allclose(result[selected], optimized)
    assert np.isfinite(result).all()
    assert np.allclose(np.linalg.det(result[:, :3, :3]), 1)


def test_new_free_model_compatible_readonly_bundle(tmp_path):
    grid = HeightGrid(.05, 4)
    grid.free[:] = True
    grid.occupied[20, 20] = True
    geometry = GeometryStore(.01)
    geometry.add(np.random.default_rng(4).normal(size=(200, 3)))
    bundle = save_bundle(tmp_path, grid, geometry, metadata(), 'flat_ground_3d_observed_rays')
    root, manifest, _, points = load_bundle(bundle)
    assert root == bundle and len(points) >= 100
    assert manifest['free_space_model'] == 'flat_ground_3d_observed_rays'
