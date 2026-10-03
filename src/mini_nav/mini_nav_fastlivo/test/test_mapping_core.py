from pathlib import Path
import numpy as np
import pytest
from mini_nav_fastlivo.geometry import collision_points, pose_matrix, planar, transform
from mini_nav_fastlivo.grid import HeightGrid, GeometryStore
from mini_nav_fastlivo.bundle import save_bundle, load_bundle


def test_unknown_rays_and_occupied_priority():
    grid = HeightGrid(0.1, 4.0)
    assert np.all(grid.data() == -1)
    grid.mark(np.array([[0.5, 0, 0.1]]))
    grid.rays([0, 0], [[1, 0]])
    cells = grid.cells([[0.5, 0], [0.2, 0], [-1, -1]])
    assert [grid.data()[y, x] for x, y in cells] == [100, 0, -1]


def test_full_height_and_self_filter():
    points = np.array([[1, 0, 0], [1, 0, 0.021], [1, 0, 0.4], [1, 0, 0.41], [0, 0, 0.1], [np.nan, 0, 0.1]])
    result = collision_points(points, np.eye(4), 0.02, 0.4)
    assert np.allclose(result, points[[1, 2]])


def test_negative_coordinates_and_bounds():
    grid = HeightGrid(0.1, 4)
    assert grid.cells([[-1.999, -1.999]]).tolist() == [[0, 0]]
    assert len(grid.cells([[-2.001, 0], [np.nan, 0]])) == 0
    assert grid.out_of_bounds


def sample():
    grid = HeightGrid(0.1, 4)
    grid.rays([0, 0], [[1, 0]])
    grid.mark(np.array([[1, 0, 0.1]]))
    store = GeometryStore(0.05)
    store.add(np.random.default_rng(3).normal(size=(200, 3)))
    alignment = {'map_world': np.eye(4).tolist(), 'imu_base': np.eye(4).tolist(),
                 'min_height': 0.02, 'max_height': 0.4, 'last_map_base': np.eye(4).tolist()}
    return grid, store, alignment


def test_bundle_roundtrip_and_read_only(tmp_path):
    grid, store, alignment = sample()
    root = save_bundle(tmp_path, grid, store, alignment)
    before = {p.name: p.read_bytes() for p in root.iterdir()}
    _, manifest, _, points = load_bundle(root)
    assert len(points) == len(store.voxels)
    assert manifest['free_space_model'] == 'flat_ground_real_scan'
    assert before == {p.name: p.read_bytes() for p in root.iterdir()}
    assert not list(tmp_path.glob('.pending_*'))


def test_bundle_tamper_and_missing_rejected(tmp_path):
    grid, store, alignment = sample()
    root = save_bundle(tmp_path, grid, store, alignment)
    (root / 'geometry.pcd').write_bytes(b'bad')
    with pytest.raises(ValueError, match='hash'):
        load_bundle(root)


def test_capacity_failure_never_commits(tmp_path):
    grid, store, alignment = sample()
    grid.out_of_bounds = True
    with pytest.raises(ValueError, match='capacity'):
        save_bundle(tmp_path, grid, store, alignment)
    assert not list(tmp_path.iterdir())


def test_pose_extrinsic_and_planar():
    matrix = pose_matrix([1, 2, 3], [0, 0, 2 ** -0.5, 2 ** -0.5])
    assert np.allclose(transform([[1, 0, 0]], matrix), [[1, 3, 3]])
    assert planar(matrix)[2, 3] == 0
    with pytest.raises(ValueError):
        pose_matrix([0, 0, 0], [0, 0, 0, 0])

def test_ground_fit_removes_height_drift_but_keeps_low_obstacle():
    from mini_nav_fastlivo.geometry import estimate_ground
    rng = np.random.default_rng(17)
    xy = rng.uniform(-2, 2, (1800, 2))
    floor = np.column_stack([xy, 0.025 + 0.007 * xy[:, 0] + rng.normal(0, 0.001, len(xy))])
    obstacle = np.array([[1.0, 0.0, 0.06], [1.0, 0.5, 0.07]])
    plane = estimate_ground(np.vstack([floor, obstacle]), np.eye(4))
    filtered = collision_points(np.vstack([floor, obstacle]), np.eye(4), 0.02, 0.4, ground_plane=plane)
    assert np.allclose(filtered, obstacle)
