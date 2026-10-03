#!/usr/bin/env python3
"""Evaluate relative IMU pose against Gazebo model truth, with timestamp matching."""
import argparse
import json
from pathlib import Path

import numpy as np
from scipy.spatial.transform import Rotation


parser = argparse.ArgumentParser()
parser.add_argument('result', type=Path)
args = parser.parse_args()
data = json.loads(args.result.read_text())
livo = np.array(data['poses']['livo'])
truth = np.array(data['poses']['truth'])
if len(livo) < 2 or len(truth) < 2:
    raise SystemExit('Insufficient fused or truth poses')
indices = np.abs(truth[:, 0, None] - livo[:, 0]).argmin(axis=0)
truth = truth[indices]
matched = np.abs(truth[:, 0] - livo[:, 0]) <= 0.026
livo, truth = livo[matched], truth[matched]
if len(livo) < 2:
    raise SystemExit('Insufficient timestamp matches within 26 ms')
r_truth = Rotation.from_quat(truth[:, 4:8])
r_livo = Rotation.from_quat(livo[:, 4:8])
imu_position = truth[:, 1:4] + r_truth.apply(np.tile([-0.032, 0.0, 0.068], (len(truth), 1)))
truth_relative = r_truth[0].inv().apply(imu_position - imu_position[0])
livo_relative = r_livo[0].inv().apply(livo[:, 1:4] - livo[0, 1:4])
error = np.linalg.norm(livo_relative - truth_relative, axis=1)
orientation_error = ((r_livo[0].inv() * r_livo).inv() * (r_truth[0].inv() * r_truth)).magnitude()
time_offsets = np.abs(truth[:, 0] - livo[:, 0])
gaps = np.diff(livo[:, 0])
report = {
    'matched_samples': len(livo), 'discarded_samples': len(matched) - int(matched.sum()),
    'max_time_match_seconds': float(time_offsets.max()),
    'position_rmse_m': float(np.sqrt(np.mean(error**2))),
    'position_max_error_m': float(error.max()), 'position_final_error_m': float(error[-1]),
    'orientation_final_error_deg': float(np.rad2deg(orientation_error[-1])),
    'livo_translation_m': livo_relative[-1].tolist(), 'truth_imu_translation_m': truth_relative[-1].tolist(),
    'livo_rotation_deg': float(np.rad2deg((r_livo[0].inv() * r_livo[-1]).magnitude())),
    'truth_rotation_deg': float(np.rad2deg((r_truth[0].inv() * r_truth[-1]).magnitude())),
    'min_pose_interval_seconds': float(gaps.min()), 'max_pose_interval_seconds': float(gaps.max()),
    'timestamps_strictly_increasing': bool((gaps > 0).all()),
    'all_pose_values_finite': bool(np.isfinite(livo).all()),
    'method': 'Nearest Gazebo model truth <=26ms, model-to-IMU extrinsic, align initial poses only; no fitted trajectory alignment',
}
args.result.with_name('metrics.json').write_text(json.dumps(report, indent=2) + '\n')
print(json.dumps(report, indent=2))
