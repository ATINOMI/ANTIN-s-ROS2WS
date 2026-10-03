import importlib.util
from pathlib import Path

import numpy as np
import pytest

source = Path(__file__).resolve().parents[1] / 'scripts' / 'map_store.py'
spec = importlib.util.spec_from_file_location('map_store', source)
module = importlib.util.module_from_spec(spec)
spec.loader.exec_module(module)


def test_voxels_retain_history_and_handle_negative_boundaries():
    cloud = module.VoxelMap(0.1)
    cloud.add([[-0.01, 0, 1, 3], [0.01, 0, 1, 4], [0.02, 0, 1, 5]])
    assert len(cloud.cells) == 2
    cloud.add([[1, 0, 2, 6], [np.inf, 0, 0, 0]])
    assert len(cloud.cells) == 3
    cloud.add([])
    assert len(cloud.cells) == 3
    assert np.isfinite(cloud.points()).all()
    assert cloud.points()[:, 2].max() == 2


def test_capacity_preserves_old_cells():
    cloud = module.VoxelMap(0.1, 1)
    cloud.add([[1, 0, 1, 2]])
    previous = cloud.points().copy()
    cloud.add([[2, 0, 2, 3]])
    assert cloud.capacity_reached
    np.testing.assert_array_equal(previous, cloud.points())


def test_pcd_checkpoint_is_complete_and_previous_sessions_survive(tmp_path):
    points = np.array([[1, -2, 3, 4], [2, 1, -1, 5]], dtype=np.float32)
    old = tmp_path / 'old.pcd'
    current = tmp_path / 'map.pcd'
    module.save_pcd(old, points)
    old_bytes = old.read_bytes()
    module.save_pcd(current, points)
    module.save_pcd(current, points[:1])
    header, payload = current.read_bytes().split(b'DATA binary\n', 1)
    assert b'POINTS 1\n' in header
    np.testing.assert_array_equal(np.frombuffer(payload, dtype='<f4').reshape(-1, 4), points[:1])
    assert old.read_bytes() == old_bytes
    assert not current.with_suffix('.pcd.tmp').exists()


@pytest.mark.parametrize('size', [0, -1, float('nan')])
def test_invalid_resolution(size):
    with pytest.raises(ValueError):
        module.VoxelMap(size)


@pytest.mark.parametrize('datatype', [module.PointField.FLOAT32, module.PointField.UINT32])
def test_camera_rgb_bits_survive_filtering_voxels_message_and_pcd(datatype, tmp_path):
    # PCL padding and opaque alpha can make a valid RGB bit pattern look like NaN.
    fields = [module.PointField(name=name, offset=offset, datatype=kind, count=1)
              for name, offset, kind in [('x', 0, module.PointField.FLOAT32),
                                        ('y', 4, module.PointField.FLOAT32),
                                        ('z', 8, module.PointField.FLOAT32), ('rgb', 16, datatype)]]
    records = np.zeros(4, dtype=module.point_cloud2.dtype_from_fields(fields, point_step=32))
    records['x'] = [-0.01, -0.02, 1, np.nan]
    records['z'] = 1
    packed = np.array([0xFFFF0000, 0xFF0000FF, 0xFFFFFFFF, 0xFF123456], dtype=np.uint32)
    if datatype == module.PointField.FLOAT32:
        records['rgb'].view(np.uint32)[:] = packed
    else:
        records['rgb'] = packed
    message = module.point_cloud2.create_cloud(module.Header(frame_id='camera_init'), fields,
                                              records, point_step=32)
    cloud = module.VoxelMap(0.1)
    cloud.add(module.read_color_points(message))
    previous = cloud.points().copy()
    cloud.add([[2, 0, 1, 0x123456], [-0.03, 0, 1, 0x00FF00]])
    points = cloud.points()
    assert len(points) == 3
    assert {int(p[3]) for p in points} == {0xFF0000, 0xFFFFFF, 0x123456}
    assert set(map(tuple, previous)).issubset(set(map(tuple, points)))
    filename = tmp_path / 'color_map.pcd'
    module.save_pcd(filename, points, colored=True)
    header, payload = filename.read_bytes().split(b'DATA binary\n', 1)
    assert b'FIELDS x y z rgb\n' in header and b'POINTS 3\n' in header
    decoded = np.frombuffer(payload, dtype='<f4').reshape(-1, 4)
    np.testing.assert_array_equal(decoded[:, :3], points[:, :3])
    np.testing.assert_array_equal(decoded.view('<u4')[:, 3], points[:, 3].astype(np.uint32))
    output_fields = [module.PointField(name=name, offset=i * 4,
                                      datatype=module.PointField.FLOAT32, count=1)
                     for i, name in enumerate(['x', 'y', 'z', 'rgb'])]
    output = module.point_cloud2.create_cloud(module.Header(frame_id='camera_init'), output_fields,
                                             module.cloud_array(points, colored=True))
    np.testing.assert_array_equal(module.read_color_points(output), points)


def test_camera_cloud_requires_rgb():
    fields = [module.PointField(name='x', offset=0, datatype=module.PointField.FLOAT32, count=1)]
    message = module.point_cloud2.create_cloud(module.Header(), fields, np.zeros((1, 1), dtype=np.float32))
    with pytest.raises(ValueError, match='rgb'):
        module.read_color_points(message)
