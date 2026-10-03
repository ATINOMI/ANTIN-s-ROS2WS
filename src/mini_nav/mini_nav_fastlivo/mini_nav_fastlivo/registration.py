"""small_gicp 只读先验图配准；额外检查几何残差与弱约束。"""
import numpy as np
from scipy.spatial import cKDTree
from .geometry import rigid, transform


class PriorMatcher:
    def __init__(self, points):
        import small_gicp
        self.gicp = small_gicp
        self.points = np.asarray(points, dtype=float)
        self.cloud, self.index = small_gicp.preprocess_points(self.points, 0.15, 15, 2)
        self.check_index = cKDTree(self.points)

    def align(self, points, initial):
        source, _ = self.gicp.preprocess_points(np.asarray(points, dtype=float), 0.15, 15, 2)
        if source.size() < 100:
            raise ValueError('Too few registration points')
        result = self.gicp.align(self.cloud, source, self.index, rigid(initial),
                                 registration_type='GICP', max_correspondence_distance=0.6,
                                 num_threads=2, max_iterations=35)
        matrix = rigid(result.T_target_source)
        sampled = np.asarray(points, dtype=float)[::max(1, len(points) // 3000)]
        distances, _ = self.check_index.query(transform(sampled, matrix), workers=1)
        inliers = distances < 0.25
        ratio = float(inliers.mean())
        rmse = float(np.sqrt(np.mean(distances[inliers] ** 2))) if inliers.any() else float('inf')
        # Hessian 仅作退化门控，不当作已标定协方差。
        eigen = np.linalg.eigvalsh(np.asarray(result.H))
        condition = float(eigen[0] / max(eigen[-1], 1e-9))
        healthy = bool(result.converged) and result.num_inliers >= 100 and ratio >= 0.65 and rmse <= 0.12 and condition > 1e-6
        return matrix, {'valid': healthy, 'inlier_ratio': ratio, 'rmse': rmse if np.isfinite(rmse) else None,
                        'condition': condition, 'inliers': int(result.num_inliers)}
