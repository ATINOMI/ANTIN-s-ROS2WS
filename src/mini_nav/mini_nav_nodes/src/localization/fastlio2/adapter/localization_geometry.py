"""Timestamped planar transforms used at the FAST-LIO2 / mini_nav boundary."""
import math
import numpy as np
from scipy.spatial.transform import Rotation

IMU_BASE = np.eye(4)
IMU_BASE[:3, 3] = [.032, 0., -.078]
IMU_LIDAR = np.array([.032, 0., .232])


def pose_matrix(pose):
    q = pose.orientation
    quaternion = np.array([q.x, q.y, q.z, q.w])
    if not np.isfinite(quaternion).all() or abs(np.linalg.norm(quaternion) - 1.) > .05:
        raise ValueError('Invalid quaternion')
    matrix = np.eye(4)
    matrix[:3, :3] = Rotation.from_quat(quaternion).as_matrix()
    matrix[:3, 3] = [pose.position.x, pose.position.y, pose.position.z]
    if not np.isfinite(matrix).all():
        raise ValueError('Non-finite pose')
    return matrix


def planar(matrix):
    yaw = math.atan2(matrix[1, 0], matrix[0, 0])
    result = np.eye(4)
    result[:3, :3] = Rotation.from_euler('z', yaw).as_matrix()
    result[:2, 3] = matrix[:2, 3]
    return result


def interpolate(history, stamp, max_gap=.12):
    for (a, first), (b, second) in zip(history, list(history)[1:]):
        if a <= stamp <= b and 0 < b - a <= max_gap:
            ratio = (stamp - a) / (b - a)
            yaw_a = math.atan2(first[1, 0], first[0, 0])
            yaw_b = math.atan2(second[1, 0], second[0, 0])
            delta = math.atan2(math.sin(yaw_b - yaw_a), math.cos(yaw_b - yaw_a))
            result = np.eye(4)
            result[:3, :3] = Rotation.from_euler('z', yaw_a + ratio * delta).as_matrix()
            result[:2, 3] = (1 - ratio) * first[:2, 3] + ratio * second[:2, 3]
            return result
    return None


def match_valid(metrics, covariance, minimum_points, minimum_ratio, maximum_residual):
    values = [metrics.get(k, float('nan')) for k in ['effective_points', 'matched_ratio', 'mean_abs_residual']]
    cov = np.asarray(covariance).reshape(6, 6)
    return (np.isfinite(values).all() and values[0] >= minimum_points and values[1] >= minimum_ratio
            and 0 <= values[2] <= maximum_residual and np.isfinite(cov).all()
            and np.all(np.diag(cov) >= 0) and max(cov[0, 0], cov[1, 1]) < .25 and cov[5, 5] < .1)
