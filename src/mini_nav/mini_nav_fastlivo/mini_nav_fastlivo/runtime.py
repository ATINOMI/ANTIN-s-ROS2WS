"""ROS 消息与独立算法之间的薄适配。"""
from collections import OrderedDict
import json
import math
import time
import numpy as np
from scipy.spatial.transform import Rotation
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import OccupancyGrid
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Header
from .geometry import pose_matrix


def ns(stamp):
    return stamp.sec * 1000000000 + stamp.nanosec


def message_pose(pose):
    p, q = pose.position, pose.orientation
    return pose_matrix([p.x, p.y, p.z], [q.x, q.y, q.z, q.w])


def message_tf(value):
    p, q = value.transform.translation, value.transform.rotation
    return pose_matrix([p.x, p.y, p.z], [q.x, q.y, q.z, q.w])


def tf_message(parent, child, stamp, matrix):
    value = TransformStamped()
    value.header.frame_id, value.child_frame_id, value.header.stamp = parent, child, stamp
    p, q = matrix[:3, 3], Rotation.from_matrix(matrix[:3, :3]).as_quat()
    value.transform.translation.x, value.transform.translation.y, value.transform.translation.z = map(float, p)
    value.transform.rotation.x, value.transform.rotation.y, value.transform.rotation.z, value.transform.rotation.w = map(float, q)
    return value


def cloud_xyz(message):
    if message.width * message.height > 100000:
        raise ValueError('Scan exceeds capacity')
    points = point_cloud2.read_points_numpy(message, field_names=['x', 'y', 'z'])
    points = np.asarray(points, dtype=float).reshape(-1, 3)
    return points[np.isfinite(points).all(axis=1)]


def cloud_message(points, frame, stamp):
    return point_cloud2.create_cloud_xyz32(Header(frame_id=frame, stamp=stamp), np.asarray(points, dtype=np.float32))


def grid_message(grid, stamp):
    message = OccupancyGrid()
    message.header = Header(frame_id='map', stamp=stamp)
    message.info.resolution = grid.resolution
    message.info.width = message.info.height = grid.width
    message.info.origin.position.x, message.info.origin.position.y = map(float, grid.origin)
    message.info.origin.orientation.w = 1.0
    message.data = grid.data().ravel().tolist()
    return message


def positive(node, name, fallback):
    value = node.declare_parameter(name, float(fallback)).value
    if not math.isfinite(value) or value <= 0:
        raise ValueError(name + ' must be finite and positive')
    return value


class ObservationCache:
    def __init__(self):
        self.poses = OrderedDict()
        self.clouds = OrderedDict()
        self.last_processed = -1

    def pose(self, message):
        if message.header.frame_id != 'camera_init' or message.child_frame_id != 'aft_mapped':
            raise ValueError('Unexpected LIVO pose frame')
        stamp = ns(message.header.stamp)
        self.poses[stamp] = (message, message_pose(message.pose.pose))
        while len(self.poses) > 100:
            self.poses.popitem(last=False)

    def cloud(self, message):
        if message.header.frame_id != 'camera_init':
            raise ValueError('Unexpected LIVO cloud frame')
        self.clouds[ns(message.header.stamp)] = message
        while len(self.clouds) > 5:
            self.clouds.popitem(last=False)

    def take(self):
        stamps = set(self.poses).intersection(self.clouds)
        stamps = [stamp for stamp in stamps if stamp > self.last_processed]
        if not stamps:
            return None
        stamp = max(stamps)
        pose, matrix = self.poses[stamp]
        cloud = self.clouds[stamp]
        self.last_processed = stamp
        return cloud, pose, matrix

    def closest(self, stamp, tolerance=50000000):
        if not self.poses:
            return None
        key = min(self.poses, key=lambda key: abs(key - stamp))
        return self.poses[key][1] if abs(key - stamp) <= tolerance else None
