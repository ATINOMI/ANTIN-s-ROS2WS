"""三维定位图与二维导航图的原子保存、只读哈希校验。"""
from pathlib import Path
import hashlib
import json
import os
import shutil
import tempfile
import uuid
import numpy as np
import yaml
from .geometry import rigid


def digest(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def write_pcd(path, points):
    points = np.asarray(points, dtype='<f4').reshape(-1, 3)
    header = (f'# .PCD v0.7\nVERSION 0.7\nFIELDS x y z\nSIZE 4 4 4\nTYPE F F F\n'
              f'COUNT 1 1 1\nWIDTH {len(points)}\nHEIGHT 1\nPOINTS {len(points)}\nDATA binary\n')
    with Path(path).open('wb') as stream:
        stream.write(header.encode())
        stream.write(points.tobytes())


def read_pcd(path):
    with Path(path).open('rb') as stream:
        fields = {}
        for _ in range(32):
            line = stream.readline().decode('ascii').strip()
            if line and not line.startswith('#'):
                key, *values = line.split()
                fields[key] = values
                if key == 'DATA':
                    break
        if fields.get('FIELDS') != ['x', 'y', 'z'] or fields.get('DATA') != ['binary'] or fields.get('SIZE') != ['4'] * 3 or fields.get('TYPE') != ['F'] * 3:
            raise ValueError('Expected bundle XYZ float32 binary PCD')
        count = int(fields['POINTS'][0])
        if not 100 <= count <= 1000000:
            raise ValueError('Invalid prior cloud capacity')
        raw = stream.read(count * 12 + 1)
        if len(raw) != count * 12:
            raise ValueError('PCD length mismatch')
        points = np.frombuffer(raw, '<f4').reshape(-1, 3).astype(float)
        if not np.isfinite(points).all():
            raise ValueError('Nonfinite prior map')
        return points


def save_bundle(root, grid, geometry, alignment, free_space_model='flat_ground_real_scan'):
    if grid.out_of_bounds or geometry.full:
        raise ValueError('Mapping capacity exceeded; incomplete map cannot be saved')
    data = grid.data()
    if not np.any(data == 0) or not np.any(data == 100) or len(geometry.voxels) < 100:
        raise ValueError('Need observed free, obstacles and enough geometry')
    root = Path(root).expanduser().resolve()
    root.mkdir(parents=True, exist_ok=True)
    temporary = Path(tempfile.mkdtemp(prefix='.pending_', dir=root))
    map_id = 'map_' + uuid.uuid4().hex[:16]
    try:
        image = np.where(data == 100, 0, np.where(data == 0, 254, 205)).astype(np.uint8)
        (temporary / 'navigation.pgm').write_bytes(f'P5\n{grid.width} {grid.width}\n255\n'.encode() + np.flipud(image).tobytes())
        map_yaml = {'image': 'navigation.pgm', 'mode': 'trinary', 'resolution': grid.resolution,
                    'origin': [*grid.origin.tolist(), 0.0], 'negate': 0, 'occupied_thresh': 0.65, 'free_thresh': 0.196}
        (temporary / 'navigation.yaml').write_text(yaml.safe_dump(map_yaml))
        write_pcd(temporary / 'geometry.pcd', geometry.points())
        np.savez_compressed(temporary / 'observations.npz', free=grid.free, occupied=grid.occupied)
        rigid(alignment['map_world'])
        (temporary / 'alignment.json').write_text(json.dumps(alignment, indent=2))
        names = ['navigation.pgm', 'navigation.yaml', 'geometry.pcd', 'observations.npz', 'alignment.json']
        manifest = {'schema': 1, 'map_id': map_id, 'frame': 'map', 'free_space_model': free_space_model,
                    'files': {name: digest(temporary / name) for name in names}}
        (temporary / 'manifest.json').write_text(json.dumps(manifest, indent=2))
        for path in temporary.iterdir():
            with path.open('rb') as stream:
                os.fsync(stream.fileno())
        destination = root / map_id
        os.rename(temporary, destination)
        directory_fd = os.open(root, os.O_RDONLY)
        try:
            os.fsync(directory_fd)
        finally:
            os.close(directory_fd)
        return destination
    except Exception:
        shutil.rmtree(temporary)
        raise


def load_bundle(root):
    root = Path(root).expanduser().resolve(strict=True)
    manifest = json.loads((root / 'manifest.json').read_text())
    required = {'navigation.pgm', 'navigation.yaml', 'geometry.pcd', 'observations.npz', 'alignment.json'}
    if manifest.get('schema') != 1 or manifest.get('frame') != 'map' or set(manifest.get('files', {})) != required or not manifest.get('map_id'):
        raise ValueError('Invalid bundle manifest')
    for name, expected in manifest['files'].items():
        path = root / name
        if path.is_symlink() or not path.is_file() or digest(path) != expected:
            raise ValueError('Map asset hash mismatch: ' + name)
    alignment = json.loads((root / 'alignment.json').read_text())
    rigid(alignment['map_world'])
    rigid(alignment['imu_base'])
    config = yaml.safe_load((root / 'navigation.yaml').read_text())
    if config['image'] != 'navigation.pgm' or config['mode'] != 'trinary' or config['origin'][2] != 0:
        raise ValueError('Invalid navigation map configuration')
    return root, manifest, alignment, read_pcd(root / 'geometry.pcd')
