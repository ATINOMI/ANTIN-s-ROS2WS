from geometry_msgs.msg import Point, Quaternion
from nav_msgs.msg import Odometry
from sensor_msgs.msg import PointCloud2
from mini_nav_fastlivo.runtime import ObservationCache


def test_cloud_waits_for_same_stamp_pose():
    cache = ObservationCache()
    cloud = PointCloud2()
    cloud.header.frame_id = 'camera_init'
    cloud.header.stamp.sec = 3
    cache.cloud(cloud)
    assert cache.take() is None
    pose = Odometry()
    pose.header.frame_id = 'camera_init'
    pose.child_frame_id = 'aft_mapped'
    pose.header.stamp.sec = 2
    pose.pose.pose.orientation.w = 1.0
    cache.pose(pose)
    assert cache.take() is None
    pose.header.stamp.sec = 3
    cache.pose(pose)
    assert cache.take()[0] is cloud
    assert cache.take() is None
