"""文件式 HBA 优化与 OctoMap 重放，生成成对的只读地图包。"""
from pathlib import Path
import argparse
import hashlib
import json
import os
import subprocess
import uuid
import numpy as np
from scipy.spatial.transform import Rotation, Slerp
from .bundle import write_pcd, save_bundle
from .geometry import rigid, transform, pose_matrix
from .grid import HeightGrid, GeometryStore


def read_session(path):
    path = Path(path).resolve(strict=True)
    metadata = json.loads((path / 'session.json').read_text())
    index = json.loads((path / 'frames.json').read_text())
    if metadata.get('schema') != 1 or metadata.get('point_frame') != 'imu' or metadata.get('error'):
        raise ValueError('Invalid or failed raw session')
    if len(index) != metadata['frames'] or not 15 <= len(index) <= 5000:
        raise ValueError('Invalid raw frame count')
    poses, stamps, clouds = [], [], []
    for number, entry in enumerate(index):
        filename = f'{number:06d}.npz'
        if entry['file'] != filename:
            raise ValueError('Raw frame index mismatch')
        frame_path = path / filename
        if frame_path.is_symlink():
            raise ValueError('Raw frame symlink')
        if hashlib.sha256(frame_path.read_bytes()).hexdigest() != entry['sha256']:
            raise ValueError('Raw frame hash mismatch')
        with np.load(frame_path, allow_pickle=False) as frame:
            stamp = int(frame['stamp_ns'])
            points = frame['points'].astype(float)
            pose = rigid(frame['world_imu'])
        if stamp != entry['stamp_ns'] or (stamps and stamp <= stamps[-1]) or points.shape != (entry['points'], 3) or not np.isfinite(points).all():
            raise ValueError('Raw timestamp or scan mismatch')
        stamps.append(stamp)
        poses.append(pose)
        clouds.append(points)
    if not np.isfinite([metadata['resolution'], metadata['map_size'], metadata['min_height'], metadata['max_height']]).all() or not 0 < metadata['min_height'] < metadata['max_height'] or metadata['resolution'] <= 0 or metadata['map_size'] <= 0:
        raise ValueError('Invalid raw map geometry')
    for name in ['map_world', 'imu_lidar', 'imu_base']:
        rigid(metadata[name])
    return metadata, np.asarray(stamps, dtype=np.int64), np.asarray(poses), clouds


def pose_rows(matrices):
    quaternions = Rotation.from_matrix(matrices[:, :3, :3]).as_quat()
    return np.column_stack([matrices[:, :3, 3], quaternions[:, [3, 0, 1, 2]]])


def matrices_from_rows(rows):
    rows = np.asarray(rows, dtype=float).reshape(-1, 7)
    return np.asarray([pose_matrix(row[:3], row[[4, 5, 6, 3]]) for row in rows])


def interpolate_corrections(stamps, original, selected, optimized):
    corrections = optimized @ np.linalg.inv(original[selected])
    if not np.isfinite(corrections).all():
        raise ValueError('Nonfinite backend correction')
    times = (stamps - stamps[0]).astype(float) / 1e9
    knots = times[selected]
    rotation = Slerp(knots, Rotation.from_matrix(corrections[:, :3, :3]))(times).as_matrix()
    translation = np.column_stack([np.interp(times, knots, corrections[:, axis, 3]) for axis in range(3)])
    interpolated = np.repeat(np.eye(4)[None], len(stamps), axis=0)
    interpolated[:, :3, :3], interpolated[:, :3, 3] = rotation, translation
    return interpolated @ original


def run_logged(command, log):
    environment = os.environ.copy()
    dependency_lib = Path(command[0]).resolve().parents[2] / 'install_mapping_deps/lib'
    environment['LD_LIBRARY_PATH'] = str(dependency_lib) + ':' + environment.get('LD_LIBRARY_PATH', '')
    # 本机 /opt/MVS 的旧 libusb 缺少 PCL 所需符号，仅离线子进程选系统库。
    system_usb = '/lib/x86_64-linux-gnu/libusb-1.0.so.0'
    if Path(system_usb).is_file():
        environment['LD_PRELOAD'] = system_usb
    with Path(log).open('w') as stream:
        subprocess.run(list(map(str, command)), stdout=stream, stderr=subprocess.STDOUT, check=True, timeout=600, env=environment)


def finalize(session, output, binaries, keyframe_step=5, voxel_size=1.0, downsample=0.03, ray_resolution=0.02, backend='probability'):
    if not isinstance(keyframe_step, int) or keyframe_step < 1 or not np.isfinite([voxel_size, downsample, ray_resolution]).all() or voxel_size <= 0 or downsample <= 0:
        raise ValueError('Invalid optimization parameters')
    metadata, stamps, original, clouds = read_session(session)
    output = Path(output).resolve()
    output.mkdir(parents=True, exist_ok=True)
    work = output / ('experiment_' + uuid.uuid4().hex[:16])
    work.mkdir()
    if backend not in ('probability', 'hba', 'pose_graph'):
        raise ValueError('Invalid offline backend')
    selected = np.unique(np.r_[np.arange(0, len(stamps), keyframe_step), len(stamps)-1]).astype(int)
    maximum_translation, maximum_rotation = 0.0, 0.0
    variants = [('original_probability', original)]
    optimized = None
    if backend == 'pose_graph':
        from .pose_graph import correct_trajectory
        selected, optimized_selected, graph_report = correct_trajectory(stamps, original, clouds)
        (work / 'pose_graph.json').write_text(json.dumps(graph_report, indent=2))
        optimized = interpolate_corrections(stamps, original, selected, optimized_selected)
        correction = optimized @ np.linalg.inv(original)
        maximum_translation = float(np.linalg.norm(correction[:, :3, 3], axis=1).max())
        maximum_rotation = float(Rotation.from_matrix(correction[:, :3, :3]).magnitude().max())
        if maximum_translation > .30 or maximum_rotation > np.deg2rad(10):
            raise ValueError('Pose graph correction exceeds short-scene gate')
        variants.append(('pose_graph_probability', optimized))
    if backend == 'hba':
        if not 35 <= len(selected) <= 500:
            raise ValueError('Select 35..500 HBA frames; adjust keyframe_step')
        hba = work / 'hba'
        (hba / 'pcd').mkdir(parents=True)
        reference = original[0]
        input_poses = np.linalg.inv(reference) @ original[selected]
        np.savetxt(hba / 'pose.json', pose_rows(input_poses), fmt='%.17g')
        for number, index in enumerate(selected):
            write_pcd(hba / 'pcd' / f'{number}.pcd', clouds[index])
        run_logged([Path(binaries) / 'hba_offline', hba, voxel_size, downsample], work / 'hba.log')
        rows = np.loadtxt(hba / 'pose.json', ndmin=2)
        if rows.shape != (len(selected), 7):
            raise ValueError('Backend frame count mismatch')
        optimized_selected = reference @ matrices_from_rows(rows)
        optimized = interpolate_corrections(stamps, original, selected, optimized_selected)
        correction = optimized @ np.linalg.inv(original)
        maximum_translation = float(np.linalg.norm(correction[:, :3, 3], axis=1).max())
        maximum_rotation = float(Rotation.from_matrix(correction[:, :3, :3]).magnitude().max())
        if maximum_translation > 0.30 or maximum_rotation > np.deg2rad(10):
            raise ValueError('Correction exceeds short-scene experiment gate; inspect hba.log')
        variants.append(('optimized_probability', optimized))
    np.savez(work / 'trajectories.npz', stamps=stamps, original=original,
             optimized=original if optimized is None else optimized, selected=selected)
    results = {}
    anchor = rigid(metadata['map_world'])
    imu_base = rigid(metadata['imu_base'])
    minimum, maximum = metadata['min_height'], metadata['max_height']
    projection = [metadata['resolution'], metadata['map_size'], minimum, maximum,
                  *np.asarray(metadata['imu_lidar'])[:3, 3], *imu_base[:3, 3], 0, 0, 1, 0]
    for name, poses in variants:
        directory = work / name
        directory.mkdir()
        # 最终重放使用所有原始扫描，不能仅使用 BA 下采样的关键帧。
        (directory / 'pcd').mkdir()
        for number, cloud in enumerate(clouds):
            write_pcd(directory / 'pcd' / f'{number}.pcd', cloud)
        map_poses = anchor @ poses
        np.savetxt(directory / 'poses_map.txt', pose_rows(map_poses), fmt='%.17g')
        np.savetxt(directory / 'projection.txt', [projection], fmt='%.17g')
        run_logged([Path(binaries) / 'replay_occupancy', directory, ray_resolution], directory / 'replay.log')
        grid = HeightGrid(metadata['resolution'], metadata['map_size'])
        data = np.fromfile(directory / 'grid.bin', dtype=np.int8)
        if len(data) != grid.width**2:
            raise ValueError('Projection dimensions mismatch')
        data = data.reshape(grid.width, grid.width)
        grid.free[:], grid.occupied[:] = data == 0, data == 100
        geometry = GeometryStore(0.05)
        for cloud, pose in zip(clouds, map_poses):
            points = transform(cloud, pose)
            base = pose @ imu_base
            geometry.add(points[np.linalg.norm(points[:, :2] - base[:2, 3], axis=1) >= 0.28])
        alignment = dict(metadata, last_map_base=(map_poses[-1] @ imu_base).tolist(),
                         first_map_base=(map_poses[0] @ imu_base).tolist(),
                         ground_plane=[0, 0, 1, 0], world_ground_z=-float(anchor[2, 3]),
                         scan_observations=len(poses), offline_backend=name,
                         hba_revision='a0cdd474996fd9bb76888c8d1839b49b70aa0818' if backend == 'hba' else None,
                         raw_session=str(Path(session).resolve()),
                         raw_index_sha256=hashlib.sha256((Path(session) / 'frames.json').read_bytes()).hexdigest(),
                         optimization_keyframe_step=keyframe_step if backend != 'pose_graph' else None,
                         trajectory_backend=backend,
                         pose_graph_mode='flat_ground_se2' if backend == 'pose_graph' else None,
                         ray_resolution=ray_resolution, ground_tolerance=0.008,
                         free_space_assumption='flat_ground_3d_observed_ray_projection; not full-height coverage proof')
        results[name] = str(save_bundle(output, grid, geometry, alignment,
                                       free_space_model='flat_ground_3d_observed_rays'))
    result = dict(work=str(work), frames=len(stamps), keyframes=len(selected), backend=backend,
                  maximum_correction_m=maximum_translation,
                  maximum_correction_deg=float(np.rad2deg(maximum_rotation)), bundles=results)
    (work / 'result.json').write_text(json.dumps(result, indent=2))
    return result


def main():
    parser = argparse.ArgumentParser(description='Generate offline HBA/OctoMap map candidates')
    parser.add_argument('session')
    parser.add_argument('--output', required=True)
    parser.add_argument('--binaries', required=True)
    parser.add_argument('--backend', choices=['probability', 'hba', 'pose_graph'], default='probability',
                        help='probability is the tested candidate; hba retains an experimental A/B')
    parser.add_argument('--keyframe-step', type=int, default=5)
    parser.add_argument('--voxel-size', type=float, default=1.0)
    parser.add_argument('--downsample', type=float, default=0.03)
    parser.add_argument('--ray-resolution', type=float, default=0.02)
    args = parser.parse_args()
    print(json.dumps(finalize(args.session, args.output, args.binaries, args.keyframe_step,
                              args.voxel_size, args.downsample, args.ray_resolution, args.backend), indent=2), flush=True)


if __name__ == '__main__':
    main()
