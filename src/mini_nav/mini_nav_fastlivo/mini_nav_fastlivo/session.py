"""完整局部扫描的有界后台记录；原始轨迹与优化工作目录分别保存。"""
from pathlib import Path
import json
import hashlib
import queue
import threading
import uuid
import numpy as np
from .geometry import rigid


class SessionWriter:
    def __init__(self, root, alignment, capacity=32):
        self.path = Path(root).expanduser().resolve() / ('session_' + uuid.uuid4().hex[:16])
        self.path.mkdir(parents=True)
        self.metadata = dict(alignment, schema=1, point_frame='imu', state='recording')
        self.queue = queue.Queue(maxsize=capacity)
        self.error = None
        self.count = 0
        self.last_stamp = -1
        self.index = []
        self._write_metadata()
        self.thread = threading.Thread(target=self._run, daemon=True)
        self.thread.start()

    def _write_metadata(self):
        self.metadata.update(frames=self.count, error=self.error)
        temporary = self.path / '.session.pending'
        temporary.write_text(json.dumps(self.metadata, indent=2))
        temporary.replace(self.path / 'session.json')

    def append(self, stamp, points_imu, world_imu):
        if self.error:
            raise ValueError(self.error)
        if stamp <= self.last_stamp:
            raise ValueError('Session stamps must increase; clock reset starts a new session')
        pose = rigid(world_imu)
        points = np.asarray(points_imu, dtype=np.float32).reshape(-1, 3)
        if not len(points) or not np.isfinite(points).all():
            raise ValueError('Invalid full local scan')
        try:
            self.queue.put_nowait((int(stamp), points.copy(), pose))
        except queue.Full:
            self.error = 'Session writer queue overflow; incomplete recording'
            raise ValueError(self.error)
        self.last_stamp = stamp

    def _run(self):
        while True:
            item = self.queue.get()
            try:
                if item is None:
                    return
                if self.error:
                    continue
                stamp, points, pose = item
                filename = f'{self.count:06d}.npz'
                np.savez(self.path / filename, stamp_ns=np.int64(stamp), points=points, world_imu=pose)
                self.index.append({'file': filename, 'stamp_ns': stamp, 'points': len(points),
                                   'sha256': hashlib.sha256((self.path / filename).read_bytes()).hexdigest()})
                self.count += 1
            except Exception as error:
                self.error = str(error)
            finally:
                self.queue.task_done()

    def checkpoint(self):
        self.queue.join()
        self._write_metadata()
        (self.path / 'frames.json').write_text(json.dumps(self.index, indent=2))
        if self.error:
            raise ValueError(self.error)
        return self.path

    def close(self):
        self.queue.put(None)
        self.thread.join()
        self.metadata['state'] = 'failed' if self.error else 'complete'
        self.checkpoint()
