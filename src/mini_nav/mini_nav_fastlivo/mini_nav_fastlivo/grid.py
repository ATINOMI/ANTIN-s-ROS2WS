"""单层平地建图：真实二维扫描确认空闲，三维碰撞点占据优先。

三维无返回不推断空闲；该模型不是完整车高体积的可观测性证明。
"""
import math
import numpy as np


class HeightGrid:
    def __init__(self, resolution=0.05, size=20.0):
        if not math.isfinite(resolution) or not math.isfinite(size) or resolution <= 0 or size <= 0:
            raise ValueError('Positive finite grid geometry required')
        self.resolution = resolution
        self.width = int(math.ceil(size / resolution))
        if self.width > 2000:
            raise ValueError('Grid capacity exceeded')
        self.origin = np.array([-self.width * resolution / 2] * 2)
        self.free = np.zeros((self.width, self.width), dtype=bool)
        self.occupied = np.zeros_like(self.free)
        self.out_of_bounds = False

    def cells(self, points):
        points = np.asarray(points, dtype=float).reshape(-1, 2)
        good = np.isfinite(points).all(axis=1)
        # 避免异常大坐标在浮点到整数转换时溢出。
        good &= (np.abs(points) < 1e6).all(axis=1)
        points = points[good]
        cells = np.floor((points - self.origin) / self.resolution).astype(int)
        inside = ((cells >= 0) & (cells < self.width)).all(axis=1)
        if not inside.all():
            self.out_of_bounds = True
        return cells[inside]

    def mark(self, points):
        cells = self.cells(np.asarray(points)[:, :2])
        if len(cells):
            self.occupied[cells[:, 1], cells[:, 0]] = True

    def rays(self, origin, endpoints):
        origin = np.asarray(origin, dtype=float)[:2]
        endpoints = np.asarray(endpoints, dtype=float).reshape(-1, 2)
        if not np.isfinite(origin).all() or not np.isfinite(endpoints).all():
            raise ValueError('Finite ray geometry required')
        for end in endpoints:
            count = max(2, int(np.linalg.norm(end - origin) / (self.resolution * 0.5)) + 1)
            if count > 5000:
                self.out_of_bounds = True
                continue
            # 命中端点由独立 marking 处理；途中 free 不擦除三维占据。
            cells = self.cells(np.linspace(origin, end, count)[:-1])
            self.free[cells[:, 1], cells[:, 0]] = True

    def data(self):
        result = np.full(self.free.shape, -1, dtype=np.int8)
        result[self.free] = 0
        result[self.occupied] = 100
        return result


class GeometryStore:
    def __init__(self, resolution=0.10, capacity=1000000):
        if not math.isfinite(resolution) or resolution <= 0 or capacity < 1:
            raise ValueError('Invalid geometry capacity')
        self.resolution, self.capacity = resolution, capacity
        self.voxels = {}
        self.full = False

    def add(self, points):
        points = np.asarray(points, dtype=float).reshape(-1, 3)
        points = points[np.isfinite(points).all(axis=1)]
        points = points[(np.abs(points) < 1e6).all(axis=1)]
        if not len(points):
            return
        keys = np.floor(points / self.resolution).astype(np.int64)
        keys, indices = np.unique(keys, axis=0, return_index=True)
        for key, index in zip(keys, indices):
            key = tuple(key)
            if key not in self.voxels:
                if len(self.voxels) >= self.capacity:
                    self.full = True
                    continue
                self.voxels[key] = points[index].copy()

    def points(self):
        return np.asarray(list(self.voxels.values()), dtype=np.float32).reshape(-1, 3)
