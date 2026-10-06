import numpy as np
import pytest
from PIL import Image
from scipy.spatial.transform import Rotation
import yaml
from mini_nav_fastlivo.static_map import align_xy, bind_static_map
from mini_nav_fastlivo.geometry import transform
from mini_nav_fastlivo.bundle import load_bundle, save_bundle, digest
from mini_nav_fastlivo.grid import HeightGrid, GeometryStore


def outline():
    x = np.linspace(-1, 2, 150)
    y = np.linspace(-1, 1, 120)
    return np.vstack([np.column_stack([x, np.full_like(x, -1)]),
                      np.column_stack([np.full_like(y, -1), y]),
                      np.column_stack([x[:80], np.full(80, 1)]),
                      np.column_stack([np.full_like(y, 2), y])])


def test_known_map_offset_and_rotation():
    target = outline()
    expected = np.eye(4)
    expected[:3, :3] = Rotation.from_euler('z', 0.035).as_matrix()
    expected[:2, 3] = [0.04, -0.025]
    source = transform(np.column_stack([target, np.zeros(len(target))]), np.linalg.inv(expected))[:, :2]
    estimated, quality = align_xy(source, target)
    # 离散边界点的最近邻可有毫米量级局部极小值，验证厘米级对齐精度。
    assert np.linalg.norm(estimated[:2, 3] - expected[:2, 3]) < 0.01
    assert abs(Rotation.from_matrix(estimated[:3, :3]).as_euler('xyz')[2] - 0.035) < 0.005
    assert quality['inlier_ratio'] > 0.99
    assert quality['rmse'] < 0.01


def test_unrelated_or_distant_map_is_rejected():
    with pytest.raises(ValueError, match='Maps do not agree'):
        align_xy(outline() + [4, 2], outline())


def test_invalid_observations_are_rejected():
    with pytest.raises(ValueError, match='100 finite'):
        align_xy(outline()[:50], outline())
    source = outline()
    source[0, 0] = np.nan
    with pytest.raises(ValueError, match='100 finite'):
        align_xy(source, outline())


def test_binding_preserves_static_pixels_and_read_only_sources(tmp_path):
    image = np.full((70, 80), 205, np.uint8)
    image[10:60, 10:70] = 254
    image[10, 10:70] = image[59, 10:70] = 0
    image[10:60, 10] = image[10:60, 69] = 0
    Image.fromarray(image).save(tmp_path / 'original.pgm')
    config = {'image': 'original.pgm', 'mode': 'trinary', 'resolution': 0.1,
              'origin': [-4.0, -3.5, 0], 'negate': 0, 'occupied_thresh': 0.65, 'free_thresh': 0.196}
    yaml_path = tmp_path / 'original.yaml'
    yaml_path.write_text(yaml.safe_dump(config))
    rows, columns = np.where(image == 0)
    points = np.column_stack([-4 + (columns + 0.5) * 0.1,
                              -3.5 + (70 - rows - 0.5) * 0.1, np.full(len(rows), 0.2)])
    grid = HeightGrid(0.1, 8)
    grid.rays([0, 0], [[1, 0]])
    grid.mark(points)
    geometry = GeometryStore(0.05)
    geometry.add(points)
    alignment = {'map_world': np.eye(4).tolist(), 'imu_base': np.eye(4).tolist(),
                 'min_height': 0.02, 'max_height': 0.4,
                 'last_map_base': np.eye(4).tolist(), 'ground_plane': [0, 0, 1, 0]}
    prior = save_bundle(tmp_path / 'priors', grid, geometry, alignment)
    before = {p.name: digest(p) for p in prior.iterdir()}
    yaml_before = yaml_path.read_bytes()
    destination, diagnostics = bind_static_map(prior, yaml_path, tmp_path / 'bound')
    _, manifest, new_alignment, new_points = load_bundle(destination)
    assert manifest['free_space_model'] == 'existing_static_2d_map'
    assert (destination / 'navigation.pgm').read_bytes() == (tmp_path / 'original.pgm').read_bytes()
    assert yaml_path.read_bytes() == yaml_before
    assert before == {p.name: digest(p) for p in prior.iterdir()}
    assert np.allclose(new_points, load_bundle(prior)[3], atol=1e-6)
    assert diagnostics['rmse'] < 1e-6
    assert new_alignment['static_navigation_map']['source_prior_map_id']
    assert not list((tmp_path / 'bound').glob('.pending_*'))
