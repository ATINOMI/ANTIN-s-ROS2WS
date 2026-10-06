"""将只读三维定位先验对齐到已有二维地图，不重新生成导航墙体。"""
import argparse
import copy
import json
import os
from pathlib import Path
import shutil
import tempfile
import uuid
import numpy as np
from PIL import Image
from scipy.optimize import least_squares
from scipy.spatial import cKDTree
from scipy.spatial.transform import Rotation
import yaml
from .bundle import digest, load_bundle, write_pcd
from .geometry import rigid, transform


def align_xy(source, target):
    """在近似共用初始坐标的地图之间估计小范围 SE(2)，拒绝不匹配地图。"""
    source, target = np.asarray(source, float), np.asarray(target, float)
    if (source.ndim != 2 or target.ndim != 2 or source.shape[1] != 2 or target.shape[1] != 2 or
            len(source) < 100 or len(target) < 100 or
            not np.isfinite(source).all() or not np.isfinite(target).all()):
        raise ValueError('Need at least 100 finite XY points in both maps')
    source = source[::max(1, len(source) // 6000)]
    tree = cKDTree(target)

    def moved(points, pose):
        c, s = np.cos(pose[2]), np.sin(pose[2])
        return points @ np.array([[c, s], [-s, c]]) + pose[:2]

    best = None
    for x in [-0.2, 0.0, 0.2]:
        for y in [-0.2, 0.0, 0.2]:
            for yaw in [-0.1, 0.0, 0.1]:
                pose = np.array([x, y, yaw])
                for _ in range(20):
                    distances, indexes = tree.query(moved(source, pose))
                    good = distances < 0.4
                    if good.sum() < 100:
                        break
                    matched = target[indexes[good]]
                    result = least_squares(lambda p: (moved(source[good], p) - matched).ravel(),
                        pose, bounds=([-0.5, -0.5, -0.2], [0.5, 0.5, 0.2]),
                        loss='soft_l1', f_scale=0.04, max_nfev=30)
                    change = np.linalg.norm(pose - result.x)
                    pose = result.x
                    if change < 1e-6:
                        break
                distances, _ = tree.query(moved(source, pose))
                score = float(np.mean(np.minimum(distances, 0.4) ** 2))
                if best is None or score < best[0]:
                    best = (score, pose, distances)
    _, pose, distances = best
    inliers = distances < 0.15
    ratio = float(inliers.mean())
    rmse = float(np.sqrt(np.mean(distances[inliers] ** 2))) if inliers.any() else float('inf')
    if ratio < 0.8 or rmse > 0.06 or np.max(np.abs(pose[:2])) >= 0.49 or abs(pose[2]) >= 0.19:
        raise ValueError('Maps do not agree within the local alignment limits; manual calibration is required')
    matrix = np.eye(4)
    matrix[:3, :3] = Rotation.from_euler('z', pose[2]).as_matrix()
    matrix[:2, 3] = pose[:2]
    return matrix, {'source_points': len(source), 'inlier_ratio': ratio, 'rmse': rmse,
                    'translation_xy': pose[:2].tolist(), 'yaw': float(pose[2]),
                    'method': 'local XY alignment; not global place recognition'}


def bind_static_map(prior_bundle, navigation_yaml, output):
    prior, manifest, alignment, points = load_bundle(prior_bundle)
    navigation_yaml = Path(navigation_yaml).expanduser().resolve(strict=True)
    config = yaml.safe_load(navigation_yaml.read_text())
    image_path = (navigation_yaml.parent / config['image']).resolve(strict=True)
    if config.get('mode') != 'trinary' or config.get('negate') != 0 or config['origin'][2] != 0:
        raise ValueError('This binding supports unrotated trinary maps with negate=0')
    resolution = float(config['resolution'])
    origin = np.asarray(config['origin'][:2], float)
    if not 0.01 <= resolution <= 0.5 or not np.isfinite(origin).all():
        raise ValueError('Invalid map resolution or origin')
    image = np.asarray(Image.open(image_path))
    if image.ndim != 2 or image.size > 16_000_000 or image_path.suffix.lower() != '.pgm':
        raise ValueError('Expected a bounded grayscale PGM navigation map')
    occupied = (255.0 - image) / 255.0 > float(config['occupied_thresh'])
    free = (255.0 - image) / 255.0 < float(config['free_thresh'])
    if not occupied.any() or not free.any():
        raise ValueError('Navigation map needs existing occupied and free cells')
    rows, columns = np.where(occupied)
    target = np.column_stack([origin[0] + (columns + 0.5) * resolution,
                             origin[1] + (image.shape[0] - rows - 0.5) * resolution])
    plane = np.asarray(alignment.get('ground_plane', [0, 0, 1, 0]), float)
    height = points @ plane[:3] + plane[3]
    source = points[(height > 0.08) & (height < 0.5), :2]
    matrix, diagnostics = align_xy(source, target)
    updated = copy.deepcopy(alignment)
    for key in ['map_world', 'first_map_base', 'last_map_base']:
        if key in updated:
            updated[key] = (matrix @ rigid(updated[key])).tolist()
    if 'ground_plane' in updated:
        updated['ground_plane'] = (np.linalg.inv(matrix).T @ plane).tolist()
    updated['static_navigation_map'] = {
        'source_yaml': str(navigation_yaml), 'source_image': str(image_path),
        'source_yaml_sha256': digest(navigation_yaml), 'source_image_sha256': digest(image_path),
        'source_prior': str(prior), 'source_prior_map_id': manifest['map_id'],
        'map_prior': matrix.tolist(), 'alignment': diagnostics}
    output = Path(output).expanduser().resolve()
    output.mkdir(parents=True, exist_ok=True)
    pending = Path(tempfile.mkdtemp(prefix='.pending_', dir=output))
    try:
        shutil.copy2(image_path, pending / 'navigation.pgm')
        bound_config = dict(config, image='navigation.pgm')
        (pending / 'navigation.yaml').write_text(yaml.safe_dump(bound_config))
        write_pcd(pending / 'geometry.pcd', transform(points, matrix))
        # 这里记录原二维地图的分类，不将它冒充本轮的扫描射线证据。
        np.savez_compressed(pending / 'observations.npz',
                            free=np.flipud(free).astype(np.uint8),
                            occupied=np.flipud(occupied).astype(np.uint8))
        (pending / 'alignment.json').write_text(json.dumps(updated, indent=2))
        names = ['navigation.pgm', 'navigation.yaml', 'geometry.pcd', 'observations.npz', 'alignment.json']
        map_id = 'map_static_' + uuid.uuid4().hex[:16]
        bound_manifest = {'schema': 1, 'map_id': map_id, 'frame': 'map',
                          'free_space_model': 'existing_static_2d_map',
                          'files': {name: digest(pending / name) for name in names}}
        (pending / 'manifest.json').write_text(json.dumps(bound_manifest, indent=2))
        load_bundle(pending)
        for path in pending.iterdir():
            with path.open('rb') as stream:
                os.fsync(stream.fileno())
        destination = output / map_id
        os.rename(pending, destination)
        return destination, diagnostics
    except Exception:
        shutil.rmtree(pending)
        raise


def main():
    parser = argparse.ArgumentParser(description='Bind an existing 2D map to a FAST-LIVO 3D prior')
    parser.add_argument('--prior-bundle', required=True)
    parser.add_argument('--map', required=True)
    parser.add_argument('--output', required=True)
    args = parser.parse_args()
    destination, diagnostics = bind_static_map(args.prior_bundle, args.map, args.output)
    print(json.dumps({'map_bundle': str(destination), 'alignment': diagnostics}, indent=2))
