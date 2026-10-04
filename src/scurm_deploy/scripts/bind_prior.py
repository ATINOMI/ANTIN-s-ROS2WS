#!/usr/bin/env python3
"""Bind a FAST-LIO2 binary XYZ-I snapshot to the existing navigation map."""
import argparse
import hashlib
import json
import shutil
from pathlib import Path

import numpy as np
from PIL import Image
from scipy.optimize import least_squares
from scipy.spatial import cKDTree
import yaml


def read_pcd(path):
    data = path.read_bytes()
    header, body = data.split(b'DATA binary\n', 1)
    if b'FIELDS x y z intensity' not in header or b'SIZE 4 4 4 4' not in header:
        raise ValueError('Expected binary float32 XYZ-I PCD')
    points = np.frombuffer(body, dtype='<f4').reshape(-1, 4).copy()
    if not np.isfinite(points).all():
        raise ValueError('Non-finite prior')
    return header, points


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('pcd', type=Path)
    parser.add_argument('--map', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    header, points = read_pcd(args.pcd)
    cfg = yaml.safe_load(args.map.read_text())
    image_path = args.map.parent / cfg['image']
    image = np.array(Image.open(image_path))
    occupied = ((image / 255.0) if cfg['negate'] else (1.0 - image / 255.0)) > cfg['occupied_thresh']
    yy, xx = np.nonzero(occupied)
    origin = np.array(cfg['origin'])
    if abs(origin[2]) > 1e-9:
        raise ValueError('This binder requires an axis-aligned occupancy map')
    target = np.c_[xx + .5, image.shape[0] - yy - .5] * cfg['resolution'] + origin[:2]
    tree = cKDTree(target)
    source = points[(points[:, 2] > .02) & (points[:, 2] < .40), :2]
    _, indices = np.unique(np.floor(source / .05).astype(int), axis=0, return_index=True)
    source = source[indices]

    def moved(p):
        c, s = np.cos(p[2]), np.sin(p[2])
        return source @ np.array([[c, s], [-s, c]]) + p[:2]

    candidates = []
    for x in [-.2, 0., .2]:
        for y in [-.2, 0., .2]:
            p = np.array([x, y, 0.])
            for _ in range(30):
                distances, matches = tree.query(moved(p))
                valid = distances < .3
                if valid.sum() < 100:
                    break
                result = least_squares(lambda v: (moved(v)[valid] - target[matches[valid]]).ravel(),
                                       p, loss='soft_l1', f_scale=.04)
                delta = np.linalg.norm(result.x - p)
                p = result.x
                if delta < 1e-7:
                    break
            distances = tree.query(moved(p))[0]
            candidates.append((float(np.mean(np.minimum(distances, .35) ** 2)), p, distances))
    _, pose, distances = min(candidates, key=lambda item: item[0])
    valid = distances < .15
    rmse = float(np.sqrt(np.mean(distances[valid] ** 2)))
    if valid.mean() < .8 or rmse > .06 or np.linalg.norm(pose[:2]) > .5 or abs(pose[2]) > .2:
        raise ValueError(f'Prior/map alignment rejected: {pose}, RMSE={rmse}, inliers={valid.mean()}')
    ground = points[points[:, 2] < -.02, :3]
    ground_z = float(np.median(ground[:, 2]))
    c, s = np.cos(pose[2]), np.sin(pose[2])
    points[:, :2] = points[:, :2] @ np.array([[c, s], [-s, c]]) + pose[:2]
    points[:, 2] -= ground_z
    args.output.mkdir(parents=True, exist_ok=True)
    (args.output / 'local_prior.pcd').write_bytes(header + b'DATA binary\n' + points.astype('<f4').tobytes())
    shutil.copyfile(image_path, args.output / 'local_map.pgm')
    cfg['image'] = 'local_map.pgm'
    (args.output / 'local_map.yaml').write_text(yaml.safe_dump(cfg, sort_keys=False))
    info = {'source_pcd': str(args.pcd.resolve()), 'source_pcd_sha256': hashlib.sha256(args.pcd.read_bytes()).hexdigest(),
            'source_map': str(args.map.resolve()), 'source_image_sha256': hashlib.sha256(image_path.read_bytes()).hexdigest(),
            'points': len(points), 'alignment_xy_yaw': pose.tolist(), 'z_offset': -ground_z,
            'initial_imu_xyz_yaw': [float(pose[0]), float(pose[1]), -ground_z, float(pose[2])],
            'wall_fit_points': len(source), 'wall_inlier_ratio': float(valid.mean()), 'wall_rmse_m': rmse,
            'scope': 'Local alignment of the saved Gazebo snapshot; not global place recognition'}
    info['files'] = {name: hashlib.sha256((args.output / name).read_bytes()).hexdigest()
                     for name in ['local_prior.pcd', 'local_map.pgm', 'local_map.yaml']}
    (args.output / 'localization_manifest.json').write_text(json.dumps(info, indent=2) + '\n')
    print(json.dumps(info, indent=2))


if __name__ == '__main__':
    main()
