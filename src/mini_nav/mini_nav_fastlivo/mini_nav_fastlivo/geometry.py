"""无 ROS 依赖的坐标变换和车体高度过滤。"""
import math
import numpy as np
from scipy.spatial.transform import Rotation


def rigid(matrix):
    matrix = np.asarray(matrix, dtype=float)
    if matrix.shape != (4, 4) or not np.isfinite(matrix).all():
        raise ValueError('Expected finite 4x4 transform')
    r = matrix[:3, :3]
    if not np.allclose(matrix[3], [0, 0, 0, 1], atol=1e-6) or not np.allclose(r.T @ r, np.eye(3), atol=1e-5) or abs(np.linalg.det(r) - 1) > 1e-5:
        raise ValueError('Invalid rigid transform')
    return matrix.copy()


def transform(points, matrix):
    return np.asarray(points, dtype=float) @ matrix[:3, :3].T + matrix[:3, 3]


def pose_matrix(position, quaternion):
    q = np.asarray(quaternion, dtype=float)
    p = np.asarray(position, dtype=float)
    if not np.isfinite(p).all() or not np.isfinite(q).all() or abs(np.linalg.norm(q) - 1) > 1e-3:
        raise ValueError('Invalid pose')
    result = np.eye(4)
    result[:3, :3] = Rotation.from_quat(q).as_matrix()
    result[:3, 3] = p
    return result


def planar(matrix):
    result = np.eye(4)
    yaw = math.atan2(matrix[1, 0], matrix[0, 0])
    result[:3, :3] = Rotation.from_euler('z', yaw).as_matrix()
    result[:2, 3] = matrix[:2, 3]
    return result


def collision_points(points_map, map_base, minimum, maximum, self_radius=0.28, ground_plane=None):
    points = np.asarray(points_map, dtype=float).reshape(-1, 3)
    finite = np.isfinite(points).all(axis=1)
    height = points[:, 2] if ground_plane is None else points @ ground_plane[:3] + ground_plane[3]
    own = np.linalg.norm(points[:, :2] - map_base[:2, 3], axis=1) < self_radius
    return points[finite & (height > minimum) & (height <= maximum) & ~own]


def estimate_ground(points, base):
    """在已找平的单层场景中以 RANSAC 拟合地面，不把低箱顶当全部地板。"""
    points = np.asarray(points, dtype=float)
    distance = np.linalg.norm(points[:, :2] - base[:2, 3], axis=1)
    candidates = points[(distance < 3.0) & (distance > 0.4) & (points[:, 2] > -0.10) & (points[:, 2] < 0.15)]
    if len(candidates) < 100:
        raise ValueError('Insufficient local ground observations')
    rng = np.random.default_rng(42)
    if len(candidates) > 3000:
        candidates = candidates[rng.choice(len(candidates), 3000, replace=False)]
    best = None
    count = 0
    for _ in range(50):
        triangle = candidates[rng.choice(len(candidates), 3, replace=False)]
        normal = np.cross(triangle[1] - triangle[0], triangle[2] - triangle[0])
        length = np.linalg.norm(normal)
        if length < 1e-8:
            continue
        normal /= length
        if normal[2] < 0:
            normal = -normal
        if normal[2] < 0.995:
            continue
        offset = -float(normal @ triangle[0])
        if abs(offset) > 0.08:
            continue
        inliers = np.abs(candidates @ normal + offset) < 0.008
        if int(inliers.sum()) > count:
            best, count = inliers, int(inliers.sum())
    if best is None or count < max(100, int(len(candidates) * 0.4)):
        raise ValueError('No reliable flat ground plane')
    selected = candidates[best]
    coefficients, *_ = np.linalg.lstsq(np.column_stack([selected[:, :2], np.ones(len(selected))]), selected[:, 2], rcond=None)
    normal = np.array([-coefficients[0], -coefficients[1], 1.0])
    scale = np.linalg.norm(normal)
    return np.r_[normal / scale, -coefficients[2] / scale]
