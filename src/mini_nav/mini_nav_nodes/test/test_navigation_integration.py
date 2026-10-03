#!/usr/bin/env python3
"""在域 217 中验证真实自研 ROS 节点，反馈来自运动学模型。

测试不连接 Gazebo 或真实底盘；启动前检查隔离域和速度话题占用情况，
拒绝向已有命令发布者或额外底盘订阅者所在的域发送测试速度。
"""
import math
import os
import pathlib
import signal
import subprocess
import sys
import time
import unittest

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy
from geometry_msgs.msg import PoseStamped, TransformStamped, TwistStamped
from nav_msgs.msg import OccupancyGrid, Path
from nav2_msgs.action import NavigateToPose
from std_msgs.msg import Bool, String
from std_srvs.srv import SetBool
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import PoseWithCovarianceStamped
from tf2_ros import TransformBroadcaster

BUILD = pathlib.Path(sys.argv.pop(1)) if len(sys.argv) > 1 and not sys.argv[1].startswith('-') else pathlib.Path('/home/a/ros2_ws/build/mini_nav_nodes')
if os.environ.get('ROS_DOMAIN_ID') != '217':
    raise RuntimeError('Navigation integration tests require ROS_DOMAIN_ID=217')


class NavigationIntegration(unittest.TestCase):
    """在隔离域 217 中驱动真实自研节点，使用运动学反馈验证导航任务闭环。
    """
    @classmethod
    def setUpClass(cls):
        """初始化整组测试共用的 ROS context。
        """
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        """关闭测试 ROS context。
        """
        rclpy.shutdown()

    def setUp(self):
        """创建探针与运动学仿真，确认域空闲后启动四个真实节点。
        """
        self.node = Node('navigation_probe_' + self._testMethodName)
        self.processes = {}
        self.files = []
        self.x = self.y = self.yaw = 0.0
        self.command = (0.0, 0.0)
        self.quality = True
        self.obstacle = None
        self.states = []
        self.guard_states = []
        self.paths = []
        self.moving = True
        self.max_replans = 0
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.map_pub = self.node.create_publisher(OccupancyGrid, '/map', latched)
        self.local_pub = self.node.create_publisher(OccupancyGrid, '/mini_nav/local_costmap', 1)
        self.scan_pub = self.node.create_publisher(LaserScan, "/scan", 10)
        self.valid_pub = self.node.create_publisher(Bool, '/mini_nav/local_costmap_valid', 1)
        self.quality_pub = self.node.create_publisher(Bool, '/mini_nav/localization_valid', 1)
        self.node.create_subscription(TwistStamped, '/cmd_vel', self.on_command, 1)
        self.node.create_subscription(String, '/mini_nav/navigation_status', lambda m: self.states.append(m.data), latched)
        self.node.create_subscription(String, '/mini_nav/velocity_guard_status', lambda m: self.guard_states.append(m.data), 1)
        self.node.create_subscription(Path, '/mini_nav/global_path', lambda m: self.paths.append(m), latched)
        self.tf = TransformBroadcaster(self.node)
        self.client = ActionClient(self.node, NavigateToPose, '/navigate_to_pose')
        self.last_tick = time.monotonic()
        self.timer = self.node.create_timer(0.025, self.tick)
        self.spin_for(0.3)
        self.assertEqual(self.node.count_publishers('/cmd_vel'), 0, 'Domain 217 is occupied; refusing to send commands')
        common = ['--ros-args']
        args = {
            'costmap_publisher': ['-p', 'enable_topic_goals:=false', '-p', 'fuse_local_obstacles:=true', '-p', 'publish_period_ms:=100'],
            'path_follower': ['-p', 'action_mode:=true', '-p', 'require_localization_quality:=true', '-p', 'cmd_vel_topic:=/mini_nav/cmd_vel_raw'],
            'navigation_manager': ['-p', 'blocked_timeout:=2.5', '-p', 'replan_interval:=0.4', '-p', 'task_timeout:=40.0'],
            'velocity_guard': []}
        for name, options in args.items():
            log = open(f'/tmp/mini_nav_integration_{self._testMethodName}_{name}.log', 'w')
            self.files.append(log)
            self.processes[name] = subprocess.Popen([str(BUILD / (name + '_node'))] + common + options, stdout=log, stderr=log)
        self.map = self.grid('map', 100, -2.5, -2.5)
        self.map_pub.publish(self.map)
        self.wait(lambda: self.client.server_is_ready(), 5)
        self.spin_for(0.7)
        self.assertEqual(self.node.count_publishers('/cmd_vel'), 1)
        self.assertEqual(self.node.count_subscribers('/cmd_vel'), 1, 'Only the test plant may receive commands')

    def tearDown(self):
        """恢复被暂停的子进程并依次结束，释放探针与日志。
        """
        for process in self.processes.values():
            if process.poll() is None:
                process.send_signal(signal.SIGCONT)
                process.send_signal(signal.SIGINT)
        for process in self.processes.values():
            try:
                process.wait(timeout=3)
            except subprocess.TimeoutExpired:
                process.kill()
                process.wait()
        for file in self.files:
            file.close()
        self.client.destroy()
        self.timer.cancel()
        self.node.destroy_node()
        time.sleep(0.15)

    def on_command(self, message):
        """缓存底盘候选速度供运动学反馈使用。
        
        Args:
            message: ROS 回调消息。
        """
        self.command = (message.twist.linear.x, message.twist.angular.z)

    def grid(self, frame, size, x, y):
        """创建 0.05 m 分辨率的全空闲占据图。
        
        Args:
            frame: 地图参考帧。
            size: 方形地图单轴格数。
            x: 位置 x，米。
            y: 位置 y，米。
        
        Returns:
            OccupancyGrid: 指定原点的全空闲地图。
        """
        message = OccupancyGrid()
        message.header.frame_id = frame
        message.header.stamp = self.node.get_clock().now().to_msg()
        message.info.width = message.info.height = size
        message.info.resolution = 0.05
        message.info.origin.position.x = x
        message.info.origin.position.y = y
        message.info.origin.orientation.w = 1.0
        message.data = [0] * (size * size)
        return message

    def tick(self):
        """推进运动学位姿并发布 TF、扫描、局部图和质量心跳。
        """
        current = time.monotonic()
        dt = min(0.05, current - self.last_tick)
        self.last_tick = current
        if self.moving:
            v, w = self.command
            self.x += v * dt * math.cos(self.yaw + w * dt / 2)
            self.y += v * dt * math.sin(self.yaw + w * dt / 2)
            self.yaw += w * dt
        stamp = self.node.get_clock().now().to_msg()
        transforms = []
        for parent, child, x, y, yaw in [('map', 'odom', 0.0, 0.0, 0.0), ('odom', 'base_footprint', self.x, self.y, self.yaw)]:
            transform = TransformStamped()
            transform.header.frame_id = parent
            transform.child_frame_id = child
            transform.header.stamp = stamp
            transform.transform.translation.x = x
            transform.transform.translation.y = y
            transform.transform.rotation.z = math.sin(yaw / 2)
            transform.transform.rotation.w = math.cos(yaw / 2)
            transforms.append(transform)
        self.tf.sendTransform(transforms)
        grid = self.grid('odom', 100, -2.5, -2.5)
        if self.obstacle:
            xmin, xmax, ymin, ymax = self.obstacle
            for y in range(100):
                for x in range(100):
                    wx, wy = -2.5 + (x + 0.5) * 0.05, -2.5 + (y + 0.5) * 0.05
                    if xmin <= wx <= xmax and ymin <= wy <= ymax:
                        grid.data[y * 100 + x] = 100
                    # Match the production local map's circular inscribed band (not soft costs).
                    elif math.hypot(max(xmin - wx, 0.0, wx - xmax), max(ymin - wy, 0.0, wy - ymax)) <= 0.28:
                        grid.data[y * 100 + x] = 99
        scan = LaserScan()
        scan.header.frame_id = 'base_footprint'
        scan.header.stamp = stamp
        scan.angle_min = -math.pi
        scan.angle_increment = 2.0 * math.pi / 360
        scan.range_min = 0.01
        scan.range_max = 3.0
        ranges = []
        for index in range(360):
            distance = float('inf')
            if self.obstacle:
                angle = self.yaw - math.pi + index * 2.0 * math.pi / 360
                dx, dy = math.cos(angle), math.sin(angle)
                tmin, tmax = 0.0, 3.0
                for position, direction, lower, upper in [(self.x, dx, xmin, xmax), (self.y, dy, ymin, ymax)]:
                    if abs(direction) < 1e-9:
                        if not lower <= position <= upper:
                            tmin, tmax = 1.0, 0.0
                            break
                    else:
                        a, b = (lower - position) / direction, (upper - position) / direction
                        tmin, tmax = max(tmin, min(a, b)), min(tmax, max(a, b))
                if tmin <= tmax:
                    distance = tmin
            ranges.append(distance)
        scan.ranges = ranges
        self.scan_pub.publish(scan)
        self.local_pub.publish(grid)
        self.valid_pub.publish(Bool(data=True))
        self.quality_pub.publish(Bool(data=self.quality))

    def spin_for(self, seconds):
        """处理 ROS 回调至给定稳态时长结束。
        
        Args:
            seconds: 稳态等待时长，秒。
        """
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            rclpy.spin_once(self.node, timeout_sec=0.01)

    def wait(self, condition, timeout=5):
        """处理回调并等待条件，超过期限令测试失败。
        
        Args:
            condition: 可重复查询的等待条件。
            timeout: 最大等待时间，秒。
        """
        end = time.monotonic() + timeout
        while time.monotonic() < end:
            if condition():
                return
            rclpy.spin_once(self.node, timeout_sec=0.01)
        self.fail(f'Timeout in {self._testMethodName}: states={self.states[-12:]}, guard={self.guard_states[-5:]}, pose={(self.x,self.y,self.yaw)}, command={self.command}')

    def send(self, x, y=0.0, yaw=0.0):
        """提交 map 系导航目标，等待接受并返回句柄与结果 future。
        
        Args:
            x: 位置 x，米。
            y: 位置 y，米。
            yaw: 偏航，弧度。
        
        Returns:
            tuple: 接受的目标句柄和异步结果 future。
        """
        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = 'map'
        goal.pose.header.stamp = self.node.get_clock().now().to_msg()
        goal.pose.pose.position.x, goal.pose.pose.position.y = float(x), float(y)
        goal.pose.pose.orientation.z = math.sin(yaw / 2)
        goal.pose.pose.orientation.w = math.cos(yaw / 2)

        def feedback(message):
            """记录任务反馈中的最大重规划次数。
            
            Args:
                message: ROS 回调消息。
            """
            self.max_replans = max(self.max_replans, message.feedback.number_of_recoveries)
        future = self.client.send_goal_async(goal, feedback_callback=feedback)
        self.wait(future.done)
        handle = future.result()
        self.assertTrue(handle.accepted)
        return handle, handle.get_result_async()

    def test_arrival_feedback_and_zero(self):
        """验证导航成功、实际终点误差及到达后的零速。
        """
        _, result = self.send(0.5)
        self.wait(result.done, 15)
        self.assertEqual(result.result().status, 4)
        endpoint = next(path.poses[-1].pose.position for path in reversed(self.paths) if path.poses)
        self.assertLess(math.hypot(self.x - endpoint.x, self.y - endpoint.y), 0.13)
        self.spin_for(0.1)
        self.assertEqual(self.command, (0.0, 0.0))

    def test_cancel_and_preempt(self):
        """验证取消及新目标抢占使旧任务退出并停车。
        """
        _, first_result = self.send(1.0)
        self.wait(lambda: self.command[0] > 0.0)
        _, second_result = self.send(0.45)
        self.wait(first_result.done)
        self.assertEqual(first_result.result().status, 6)
        self.assertEqual(first_result.result().result.error_msg, 'preempted')
        self.wait(second_result.done, 15)
        self.assertEqual(second_result.result().status, 4)
        third, third_result = self.send(1.0)
        self.wait(lambda: self.command[0] > 0.0)
        cancel = third.cancel_goal_async()
        self.wait(cancel.done)
        self.wait(third_result.done)
        self.assertEqual(third_result.result().status, 5)
        self.spin_for(0.15)
        self.assertEqual(self.command, (0.0, 0.0))

    def test_navigation_disable_rejects_goals_and_requires_new_task(self):
        """验证暂停拒绝目标，启用后须提交新任务。
        """
        _, result = self.send(1.0)
        self.wait(lambda: self.command[0] > 0.0)
        service = self.node.create_client(SetBool, '/mini_nav/set_navigation_enabled')
        self.wait(service.service_is_ready)
        future = service.call_async(SetBool.Request(data=False))
        self.wait(future.done)
        self.assertTrue(future.result().success)
        self.wait(result.done)
        self.assertEqual(result.result().result.error_msg, 'navigation_disabled')
        self.spin_for(0.1)
        self.assertEqual(self.command, (0.0, 0.0))
        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = 'map'
        goal.pose.pose.position.x = 0.5
        goal.pose.pose.orientation.w = 1.0
        rejected = self.client.send_goal_async(goal)
        self.wait(rejected.done)
        self.assertFalse(rejected.result().accepted)
        future = service.call_async(SetBool.Request(data=True))
        self.wait(future.done)
        self.spin_for(0.2)
        self.assertEqual(self.command, (0.0, 0.0))
        _, new_result = self.send(0.5)
        self.wait(new_result.done, 15)
        self.assertEqual(new_result.result().status, 4)
        service.destroy()

    def test_map_change_and_localization_reset_terminate_old_tasks(self):
        """验证地图更换和重定位终止旧任务。
        """
        _, result = self.send(1.0)
        self.wait(lambda: self.command[0] > 0.0)
        self.map.data[100] = 100
        self.map.header.stamp = self.node.get_clock().now().to_msg()
        self.map_pub.publish(self.map)
        self.wait(result.done)
        self.assertEqual(result.result().result.error_msg, 'map_changed')
        self.spin_for(0.1)
        self.assertEqual(self.command, (0.0, 0.0))
        _, result = self.send(1.0)
        self.wait(lambda: self.command[0] > 0.0)
        reset_pub = self.node.create_publisher(PoseWithCovarianceStamped, '/initialpose', 10)
        self.spin_for(0.1)
        initial = PoseWithCovarianceStamped()
        initial.header.frame_id = 'map'
        initial.pose.pose.orientation.w = 1.0
        reset_pub.publish(initial)
        self.wait(result.done)
        self.assertEqual(result.result().result.error_msg, 'localization_reset')
        self.spin_for(0.2)
        self.assertEqual(self.command, (0.0, 0.0))

    def test_localization_epoch_cancels_task_without_automatic_resume(self):
        """定位会话改变终止旧任务，质量恢复不续跑。"""
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        publisher = self.node.create_publisher(String, '/mini_nav/localization_epoch', latched)
        publisher.publish(String(data='map/session/0'))
        self.spin_for(0.3)
        _, result = self.send(1.0)
        self.wait(lambda: self.command[0] > 0.0)
        publisher.publish(String(data='map/session/1'))
        self.wait(result.done)
        self.assertEqual(result.result().result.error_msg, 'localization_epoch_changed')
        self.spin_for(0.5)
        self.assertEqual(self.command, (0.0, 0.0))

    def test_dynamic_obstacle_replan_and_arrival(self):
        """验证新增动态障碍触发重规划并最终到达。
        """
        _, result = self.send(1.2)
        self.wait(lambda: self.x > 0.04)
        self.obstacle = (0.5, 0.6, -0.15, 0.15)
        self.wait(lambda: self.max_replans > 0, 5)
        self.wait(lambda: any(any(abs(p.pose.position.y) > 0.4 for p in path.poses) for path in self.paths), 5)
        self.wait(result.done, 35)
        self.assertEqual(result.result().status, 4)
        endpoint = next(path.poses[-1].pose.position for path in reversed(self.paths) if path.poses)
        self.assertLess(math.hypot(self.x - endpoint.x, self.y - endpoint.y), 0.13)

    def test_persistent_obstacle_has_finite_failure(self):
        """验证持续受阻在有限期限内失败并停车。
        """
        self.obstacle = (0.5, 0.6, -2.3, 2.3)
        self.spin_for(0.15)
        _, result = self.send(1.2)
        self.wait(result.done, 7)
        self.assertEqual(result.result().status, 6)
        self.assertIn(result.result().result.error_msg, ['no_progress', 'blocked_timeout'])
        self.assertEqual(self.command, (0.0, 0.0))

    def test_localization_loss_stops_and_fails(self):
        """验证定位质量丢失导致停车及任务失败。
        """
        _, result = self.send(1.0)
        self.wait(lambda: self.command[0] > 0.0)
        self.quality = False
        self.spin_for(0.3)
        self.assertEqual(self.command, (0.0, 0.0))
        self.wait(result.done, 7)
        self.assertEqual(result.result().status, 6)

    def test_frozen_controller_and_manager_stop_independently(self):
        """验证控制器或任务节点冻结时独立看门狗仍停车。
        """
        _, _ = self.send(1.2)
        self.wait(lambda: self.command[0] > 0.0)
        self.processes['path_follower'].send_signal(signal.SIGSTOP)
        self.wait(lambda: self.command == (0.0, 0.0), 0.7)
        self.assertIn('command_stale', self.guard_states)
        self.processes['path_follower'].send_signal(signal.SIGCONT)
        self.wait(lambda: self.command[0] > 0.0, 2)
        self.processes['navigation_manager'].send_signal(signal.SIGSTOP)
        self.wait(lambda: self.command == (0.0, 0.0), 0.7)
        self.assertIn('task_inactive', self.guard_states)
        self.spin_for(0.2)
        self.assertEqual(self.command, (0.0, 0.0))


class VelocityGuardIntegration(unittest.TestCase):
    """在隔离域中验证独立看门狗的仿真时钟与非法命令保护。
    """
    def test_clock_stall_and_invalid_command_cannot_reach_base(self):
        """验证仿真时钟停滞、旧时间戳和运动许可撤销均阻止速度到达底盘。
        """
        rclpy.init()
        node = Node('velocity_guard_clock_probe')
        command = [None]
        reasons = []
        node.create_subscription(TwistStamped, '/cmd_vel', lambda m: command.__setitem__(0, m.twist.linear.x), 1)
        node.create_subscription(String, '/mini_nav/velocity_guard_status', lambda m: reasons.append(m.data), 1)
        clock_pub = node.create_publisher(Clock, '/clock', 1)
        raw_pub = node.create_publisher(TwistStamped, '/mini_nav/cmd_vel_raw', 1)
        lease_pub = node.create_publisher(Bool, '/mini_nav/task_active', 1)
        process = None
        paused = False
        invalid = False
        active = True
        sim_time = 10.0
        last = time.monotonic()

        def tick():
            """持续发布可控制停滞、时间戳异常和许可失效的看门狗测试输入。
            """
            nonlocal sim_time, last
            current = time.monotonic()
            if not paused:
                sim_time += current - last
            last = current
            clock = Clock()
            clock.clock.sec = int(sim_time)
            clock.clock.nanosec = int((sim_time - int(sim_time)) * 1e9)
            clock_pub.publish(clock)
            raw = TwistStamped()
            raw.header.frame_id = 'base_footprint'
            raw.header.stamp = clock.clock
            if invalid:
                raw.header.stamp.sec -= 2
            raw.twist.linear.x = 0.1
            raw_pub.publish(raw)
            lease_pub.publish(Bool(data=active))

        timer = node.create_timer(0.025, tick)

        def wait(predicate, timeout=3):
            """处理回调并等待条件，超过期限令测试失败。
            
            Args:
                predicate: 等待条件。
                timeout: 最大等待时间，秒。
            """
            end = time.monotonic() + timeout
            while time.monotonic() < end:
                rclpy.spin_once(node, timeout_sec=0.01)
                if predicate():
                    return
            self.fail(f'Watchdog failed: {command}, {reasons[-10:]}')

        try:
            end = time.monotonic() + 0.3
            while time.monotonic() < end:
                rclpy.spin_once(node, timeout_sec=0.01)
            self.assertEqual(node.count_publishers('/cmd_vel'), 0)
            with open('/tmp/mini_nav_guard_clock_test.log', 'w') as log:
                process = subprocess.Popen([str(BUILD / 'velocity_guard_node'), '--ros-args', '-p', 'use_sim_time:=true'], stdout=log, stderr=log)
                wait(lambda: command[0] == 0.1)
                self.assertEqual(node.count_subscribers('/cmd_vel'), 1)
                paused = True
                wait(lambda: command[0] == 0.0 and reasons[-1] == 'clock_stale', 0.8)
                paused = False
                wait(lambda: command[0] == 0.1)
                invalid = True
                wait(lambda: command[0] == 0.0 and reasons[-1] == 'invalid_command')
                invalid = False
                wait(lambda: command[0] == 0.1)
                active = False
                wait(lambda: command[0] == 0.0 and reasons[-1] == 'task_inactive')
                active = True
                wait(lambda: command[0] == 0.1)
                process.send_signal(signal.SIGINT)
                wait(lambda: command[0] == 0.0)
                process.wait(timeout=3)
        finally:
            if process is not None and process.poll() is None:
                process.send_signal(signal.SIGINT)
                try:
                    process.wait(timeout=3)
                except subprocess.TimeoutExpired:
                    process.kill()
                    process.wait()
            timer.cancel()
            node.destroy_node()
            rclpy.shutdown()


if __name__ == '__main__':
    unittest.main(verbosity=2)
