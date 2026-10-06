"""平地离线位姿图：真实扫描约束校正历史轨迹，原始记录保持只读。"""
import numpy as np
from scipy.optimize import least_squares
from scipy.sparse import lil_matrix
from scipy.spatial.transform import Rotation
from .registration import PriorMatcher


def wrap(angle):
    return np.arctan2(np.sin(angle), np.cos(angle))


def planar_rows(poses):
    return np.column_stack([poses[:, :2, 3],
        np.arctan2(poses[:, 1, 0], poses[:, 0, 0])])


def relative(first, second):
    c, s = np.cos(first[2]), np.sin(first[2])
    delta = second[:2] - first[:2]
    return np.array([c*delta[0]+s*delta[1], -s*delta[0]+c*delta[1],
                     wrap(second[2]-first[2])])


def select_keyframes(stamps, poses):
    rows = planar_rows(poses)
    selected = [0]
    for i in range(1, len(rows)):
        previous = selected[-1]
        if (np.linalg.norm(rows[i, :2]-rows[previous, :2]) >= .15 or
                abs(wrap(rows[i, 2]-rows[previous, 2])) >= .25 or
                stamps[i]-stamps[previous] >= 5_000_000_000):
            selected.append(i)
    if selected[-1] != len(rows)-1:
        selected.append(len(rows)-1)
    return np.asarray(selected, dtype=int)


def optimize_graph(initial, edges):
    """固定首节点消除坐标自由度；边保存 source→target 的相对位姿。"""
    initial = np.asarray(initial, dtype=float)
    if len(initial) < 2 or not np.isfinite(initial).all():
        raise ValueError('Invalid pose graph nodes')
    sparsity = lil_matrix((3*len(edges), 3*(len(initial)-1)), dtype=int)
    for number, edge in enumerate(edges):
        for node in [edge['i'], edge['j']]:
            if not 0 <= node < len(initial):
                raise ValueError('Invalid edge node')
            if node:
                sparsity[3*number:3*number+3, 3*(node-1):3*node] = 1

    def residual(values):
        rows = np.vstack([initial[0], values.reshape(-1, 3)])
        errors = []
        for edge in edges:
            error = relative(rows[edge['i']], rows[edge['j']])-edge['measurement']
            error[2] = wrap(error[2])
            errors.extend(error/np.asarray(edge['sigma']))
        return np.asarray(errors)

    solved = least_squares(residual, initial[1:].ravel(), jac_sparsity=sparsity,
                           loss='huber', f_scale=2., max_nfev=200)
    if not solved.success or not np.isfinite(solved.x).all():
        raise ValueError('Pose graph did not converge')
    rows = np.vstack([initial[0], solved.x.reshape(-1, 3)])
    return rows, {'cost_before': float(np.sum(residual(initial[1:].ravel())**2)),
                  'cost_after': float(np.sum(residual(solved.x)**2)),
                  'solver_message': solved.message}


def match_loop(target, source, initial):
    matcher = PriorMatcher(target)
    matrix, quality = matcher.align(source, initial)
    if not quality['valid'] or quality['inlier_ratio'] < .80 or quality['rmse'] > .05 or quality['condition'] < 1e-4:
        return None
    reverse, reverse_quality = PriorMatcher(source).align(target, np.linalg.inv(matrix))
    cycle = reverse @ matrix
    if (not reverse_quality['valid'] or reverse_quality['inlier_ratio'] < .80 or
            reverse_quality['rmse'] > .05 or np.linalg.norm(cycle[:3, 3]) > .03 or
            Rotation.from_matrix(cycle[:3, :3]).magnitude() > .02):
        return None
    delta = matrix @ np.linalg.inv(initial)
    tilt = Rotation.from_matrix(delta[:3, :3]).as_rotvec()
    if abs(delta[2, 3]) > .06 or np.linalg.norm(tilt[:2]) > .05:
        return None
    return matrix, dict(quality, reverse_rmse=reverse_quality['rmse'])


def correct_trajectory(stamps, poses, clouds):
    selected = select_keyframes(stamps, poses)
    if len(selected) > 400:
        raise ValueError('Offline graph supports at most 400 motion keyframes')
    rows = planar_rows(poses[selected])
    travel = np.r_[0., np.cumsum(np.linalg.norm(np.diff(poses[:, :2, 3], axis=0), axis=1))]
    edges = [dict(i=i-1, j=i, measurement=relative(rows[i-1], rows[i]),
                  sigma=[.03, .03, .02], kind='odometry') for i in range(1, len(rows))]
    loops, rejected, last_loop = [], 0, -1
    for j in range(1, len(selected)):
        current = selected[j]
        if last_loop >= 0 and travel[current]-travel[selected[last_loop]] < .30:
            continue
        candidates = [i for i in range(j) if
            stamps[current]-stamps[selected[i]] >= 15_000_000_000 and
            travel[current]-travel[selected[i]] >= 1.0 and
            np.linalg.norm(rows[i, :2]-rows[j, :2]) <= .75]
        candidates.sort(key=lambda i: np.linalg.norm(rows[i, :2]-rows[j, :2]))
        for i in candidates[:2]:
            first = selected[i]
            target = clouds[first][np.linalg.norm(clouds[first][:, :2], axis=1) > .28]
            source = clouds[current][np.linalg.norm(clouds[current][:, :2], axis=1) > .28]
            initial = np.linalg.inv(poses[first]) @ poses[current]
            matched = match_loop(target, source, initial)
            if matched is None:
                rejected += 1
                continue
            matrix, quality = matched
            # 保留原始 roll/pitch/z，只校正平地导航所需的 x/y/yaw。
            corrected_current = poses[first] @ matrix
            measurement = relative(rows[i], planar_rows(corrected_current[None])[0])
            edge = dict(i=i, j=j, measurement=measurement, sigma=[.01, .01, .01], kind='loop')
            edges.append(edge)
            loops.append(dict(i=int(first), j=int(current), **quality))
            last_loop = j
            break
    if not loops:
        raise ValueError('No verified revisit constraints; trajectory was not corrected')
    optimized, solver = optimize_graph(rows, edges)
    for edge in edges:
        if edge['kind'] == 'loop':
            error = relative(optimized[edge['i']], optimized[edge['j']])-edge['measurement']
            if np.linalg.norm(error[:2]) > .05 or abs(wrap(error[2])) > .04:
                raise ValueError('Conflicting revisit constraints')
    corrected = poses[selected].copy()
    for k, (before, after) in enumerate(zip(rows, optimized)):
        rotation = Rotation.from_euler('z', wrap(after[2]-before[2])).as_matrix()
        corrected[k, :3, :3] = rotation @ corrected[k, :3, :3]
        corrected[k, :2, 3] = after[:2]
    report = dict(mode='flat_ground_se2', loops=loops, rejected_matches=rejected,
        selected=selected.tolist(), **solver,
        edges=[dict(edge, measurement=np.asarray(edge['measurement']).tolist()) for edge in edges])
    return selected, corrected, report
