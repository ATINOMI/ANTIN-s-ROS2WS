"""用真实 ROS 节点验证 learning 膨胀与车体安全图同时发布且失效一致。"""
import os
from pathlib import Path
import signal
import subprocess
import sys
import tempfile
import time
import unittest

import rclpy
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import OccupancyGrid
from mini_nav_nodes.msg import CollisionMap
from rclpy.qos import DurabilityPolicy, QoSProfile
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool
from tf2_ros import TransformBroadcaster


BUILD = Path(sys.argv.pop(1))
CONFIG = Path(__file__).resolve().parents[2] / 'mini_nav_bringup' / 'config'


class CostmapSafety(unittest.TestCase):
    """在独立域只运行地图节点，不启动控制器或发布速度。"""

    def setUp(self):
        rclpy.init()
        self.node = rclpy.create_node('costmap_safety_test')
        self.processes = []
        self.files = []
        self.temp = tempfile.TemporaryDirectory(prefix='mini_nav_costmap_safety_')
        self.messages = {}
        self.subscriptions = []
        self.latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.map_pub = self.node.create_publisher(OccupancyGrid, '/map', self.latched)
        self.scan_pub = self.node.create_publisher(LaserScan, '/scan', 10)
        self.tf = TransformBroadcaster(self.node)
        self.rays_enabled = False
        self.timer = self.node.create_timer(0.025, self.tick)
        self.spin_for(0.2)
        for topic in ['/cmd_vel', '/mini_nav/planning_costmap', '/mini_nav/local_costmap']:
            self.assertEqual(self.node.count_publishers(topic), 0, f'Isolated domain occupied: {topic}')

    def tearDown(self):
        for process in self.processes:
            if process.poll() is None:
                process.send_signal(signal.SIGINT)
        for process in self.processes:
            try:
                process.wait(timeout=5)
            except subprocess.TimeoutExpired:
                process.kill()
                process.wait()
        for stream in self.files:
            stream.close()
        self.node.destroy_node()
        rclpy.shutdown()
        self.temp.cleanup()

    def subscribe(self, topic, latched=False, message_type=OccupancyGrid):
        qos = self.latched if latched else 1
        self.subscriptions.append(self.node.create_subscription(
            message_type, topic, lambda message: self.messages.__setitem__(topic, message), qos))

    def start(self, executable, config, overrides=()):
        stream = open(Path(self.temp.name) / f'{executable}.log', 'w')
        self.files.append(stream)
        args = [str(BUILD / executable), '--ros-args', '--params-file', str(CONFIG / config)]
        for parameter in overrides:
            args.extend(['-p', parameter])
        self.processes.append(subprocess.Popen(args, stdout=stream, stderr=stream))

    def spin_for(self, seconds):
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            rclpy.spin_once(self.node, timeout_sec=0.02)

    def wait(self, condition, seconds=8):
        end = time.monotonic() + seconds
        while time.monotonic() < end and not condition():
            rclpy.spin_once(self.node, timeout_sec=0.02)
        self.assertTrue(condition(), 'Expected map/validity messages were not received')

    def tick(self):
        stamp = self.node.get_clock().now().to_msg()
        transform = TransformStamped()
        transform.header.stamp = stamp
        transform.header.frame_id = 'odom'
        transform.child_frame_id = 'base_footprint'
        transform.transform.rotation.w = 1.0
        self.tf.sendTransform(transform)
        if self.rays_enabled:
            scan = LaserScan()
            scan.header.stamp = stamp
            scan.header.frame_id = 'base_footprint'
            scan.angle_min = 0.0
            scan.angle_max = 0.0
            scan.angle_increment = 0.01
            scan.range_min = 0.01
            scan.range_max = 3.0
            scan.ranges = [0.8]
            self.scan_pub.publish(scan)

    def test_global_keeps_body_band_separate_from_display(self):
        display_topic = '/mini_nav/planning_costmap'
        safety_topic = '/mini_nav/planning_safety_costmap'
        self.subscribe(display_topic, True)
        self.subscribe(safety_topic, True)
        self.subscribe('/mini_nav/static_collision_map', True, CollisionMap)
        self.start('costmap_publisher_node', 'planning_costmap.yaml',
                   ['enable_topic_goals:=false', 'fuse_local_obstacles:=false', 'publish_period_ms:=100'])
        raw = OccupancyGrid()
        raw.header.frame_id = 'map'
        raw.info.width = raw.info.height = 31
        raw.info.resolution = 0.05
        raw.info.origin.orientation.w = 1.0
        raw.data = [0] * (31 * 31)
        raw.data[15 * 31 + 15] = 100
        self.map_pub.publish(raw)
        self.wait(lambda: display_topic in self.messages and safety_topic in self.messages)
        display, safety = self.messages[display_topic], self.messages[safety_topic]
        # 0.25 m 在官方内切圈外，但仍在真实车体 0.26 m 安全圈内。
        index = 15 * 31 + 20
        self.assertLess(display.data[index], 99)
        self.assertEqual(safety.data[index], 99)
        self.assertEqual(display.data[15 * 31 + 15], 100)
        self.assertEqual(display.info, safety.info)
        self.wait(lambda: '/mini_nav/static_collision_map' in self.messages)
        collision = self.messages['/mini_nav/static_collision_map']
        self.assertTrue(collision.valid)
        self.assertAlmostEqual(collision.clearance_radius, .29)
        self.assertEqual(collision.grid.data, raw.data)
        self.assertEqual(list(collision.points), [])
        self.assertEqual(self.node.count_publishers('/cmd_vel'), 0)

    def test_local_uses_same_raw_obstacles_and_invalidates_both_maps(self):
        display_topic = '/mini_nav/local_costmap'
        safety_topic = '/mini_nav/local_safety_costmap'
        valid_topic = '/mini_nav/local_costmap_valid'
        self.subscribe(display_topic)
        self.subscribe(safety_topic)
        self.subscribe(valid_topic, message_type=Bool)
        self.subscribe('/mini_nav/local_collision_map', message_type=CollisionMap)
        self.start('local_costmap_node', 'local_costmap.yaml')
        self.rays_enabled = True
        self.wait(lambda: display_topic in self.messages and safety_topic in self.messages and
                  valid_topic in self.messages and self.messages[valid_topic].data)
        display, safety = self.messages[display_topic], self.messages[safety_topic]
        self.assertEqual(display.info, safety.info)
        sources = [index for index, cost in enumerate(display.data) if cost == 100]
        self.assertEqual(len(sources), 1)
        index = sources[0] - 5
        self.assertLess(display.data[index], 99)
        self.assertGreaterEqual(display.data[index], 0)
        self.assertEqual(safety.data[index], 99)
        self.wait(lambda: '/mini_nav/local_collision_map' in self.messages and
                  self.messages['/mini_nav/local_collision_map'].valid)
        collision = self.messages['/mini_nav/local_collision_map']
        self.assertAlmostEqual(collision.clearance_radius, .26)
        self.assertAlmostEqual(collision.observation_uncertainty, .03)
        self.assertEqual(len(collision.points), 1)
        self.assertAlmostEqual(collision.points[0].x, .8, places=6)
        self.assertEqual(collision.grid.data[sources[0]], 0)
        self.assertIn(-1, collision.grid.data)
        self.rays_enabled = False
        self.spin_for(.1)
        frozen_stamp = self.messages['/mini_nav/local_collision_map'].grid.header.stamp
        self.spin_for(.3)
        self.assertEqual(self.messages['/mini_nav/local_collision_map'].grid.header.stamp, frozen_stamp)
        self.wait(lambda: not self.messages[valid_topic].data and
                  all(cost == -1 for cost in self.messages[display_topic].data) and
                  all(cost == -1 for cost in self.messages[safety_topic].data) and
                  not self.messages['/mini_nav/local_collision_map'].valid, seconds=4)
        self.assertEqual(self.node.count_publishers('/cmd_vel'), 0)
        self.assertFalse(self.messages['/mini_nav/local_collision_map'].valid)
        self.assertEqual(list(self.messages['/mini_nav/local_collision_map'].points), [])


if __name__ == '__main__':
    os.environ['ROS_DOMAIN_ID'] = '216'
    os.environ['RMW_IMPLEMENTATION'] = 'rmw_cyclonedds_cpp'
    os.environ['ROS_AUTOMATIC_DISCOVERY_RANGE'] = 'LOCALHOST'
    unittest.main()
