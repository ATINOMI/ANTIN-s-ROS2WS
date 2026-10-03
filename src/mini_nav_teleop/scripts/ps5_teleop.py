#!/usr/bin/env python3
"""Bridge the user's USB DualForge callbacks to the existing velocity guard."""
import math
import json
import signal
import threading
import time

import rclpy
from geometry_msgs.msg import TwistStamped
from rclpy.clock import Clock, ClockType
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.signals import SignalHandlerOptions
from std_msgs.msg import Bool
from std_msgs.msg import String
from std_srvs.srv import SetBool
from rcl_interfaces.msg import ParameterDescriptor, SetParametersResult
from rclpy.parameter import Parameter
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy


def axis_command(raw, deadzone):
    if not isinstance(raw, int) or isinstance(raw, bool) or not 0 <= raw <= 255:
        raise ValueError('Stick value must be an unsigned byte')
    value = (128 - raw) / (128 if raw <= 128 else 127)
    if abs(value) <= deadzone:
        return 0.0
    return math.copysign((abs(value) - deadzone) / (1 - deadzone), value)


def command_from_state(state, linear_limit=2.0, angular_limit=2.0, deadzone=0.12, enabled=False):
    try:
        if not enabled:
            return 0.0, 0.0, False
        # USB stick up is smaller Y; right-stick left is smaller X.
        linear = axis_command(state['sticks']['left']['y'], deadzone) * linear_limit
        angular = axis_command(state['sticks']['right']['x'], deadzone) * angular_limit
    except (KeyError, TypeError, ValueError):
        return 0.0, 0.0, False
    return linear, angular, True


class InputBuffer:
    def __init__(self, timeout, linear_limit, angular_limit, deadzone):
        self.lock = threading.Lock()
        self.connected = False
        self.received = None
        self.command = (0.0, 0.0, False)
        self.enabled = False
        self.previous_l1 = None
        self.blocked = False
        self.axes = None
        self.timeout = timeout
        self.limits = (linear_limit, angular_limit, deadzone)

    def connection(self, connected):
        with self.lock:
            self.connected = bool(connected)
            self.received = None
            self.command = (0.0, 0.0, False)
            self.enabled = False
            self.previous_l1 = None
            self.axes = None

    def disable(self):
        with self.lock:
            self.enabled = False
            self.command = (0.0, 0.0, False)

    def set_enabled(self, enabled, current=None):
        current = time.monotonic() if current is None else current
        with self.lock:
            if not enabled:
                self.enabled = False
                self.command = (0.0, 0.0, False)
                return True, 'Control disabled'
            if (not self.connected or self.axes is None or self.received is None
                    or not 0 <= current - self.received <= self.timeout):
                return False, 'Fresh USB input required'
            if self.blocked:
                return False, 'Clock or control publishers unavailable'
            self.enabled = True
            self.command = (self.axes[0] * self.limits[0], self.axes[1] * self.limits[1], True)
            return True, 'Control enabled'

    def set_limits(self, linear, angular):
        with self.lock:
            self.limits = (linear, angular, self.limits[2])
            if self.enabled and self.axes is not None:
                self.command = (self.axes[0] * linear, self.axes[1] * angular, True)

    def gate(self, blocked):
        with self.lock:
            self.blocked = blocked
            if blocked:
                self.enabled = False
                self.command = (0.0, 0.0, False)

    def receive(self, state, received=None):
        current = time.monotonic() if received is None else received
        command = command_from_state(state, 1.0, 1.0, self.limits[2], enabled=True)
        try:
            pressed = state['buttons']['l1']
            valid = isinstance(pressed, bool) and command[2]
        except (KeyError, TypeError):
            valid = False
        with self.lock:
            if self.connected:
                if self.received is not None and not 0 <= current - self.received <= self.timeout:
                    self.enabled = False
                    self.previous_l1 = None
                if not valid:
                    self.enabled = False
                    self.previous_l1 = None
                    self.axes = None
                else:
                    self.axes = command[:2]
                    if not self.blocked and self.previous_l1 is not None and pressed and not self.previous_l1:
                        self.enabled = not self.enabled
                    self.previous_l1 = pressed
                self.command = ((command[0] * self.limits[0], command[1] * self.limits[1], True)
                                if self.enabled else (0.0, 0.0, False))
                self.received = current

    def snapshot(self, current=None):
        current = time.monotonic() if current is None else current
        with self.lock:
            if not self.connected or self.received is None or not 0 <= current - self.received <= self.timeout:
                self.enabled = False
                self.previous_l1 = None
                self.command = (0.0, 0.0, False)
                return 0.0, 0.0, False
            return self.command


class Ps5Teleop(Node):
    def __init__(self, controller=None):
        super().__init__('ps5_teleop')
        linear = self.declare_parameter('linear_limit', 2.0).value
        angular = self.declare_parameter('angular_limit', 2.0).value
        readonly = ParameterDescriptor(read_only=True)
        deadzone = self.declare_parameter('deadzone', 0.12, readonly).value
        timeout = self.declare_parameter('input_timeout', 0.25, readonly).value
        for value, maximum, name in [(linear, 2.0, 'linear_limit'), (angular, 2.0, 'angular_limit'),
                                     (deadzone, 0.4, 'deadzone'), (timeout, 0.30, 'input_timeout')]:
            minimum_valid = value >= 0 if name.endswith('_limit') else value > 0
            if not math.isfinite(value) or not minimum_valid or value > maximum:
                raise ValueError(f'{name} exceeds allowed range')
        self.buffer = InputBuffer(timeout, linear, angular, deadzone)
        self.add_on_set_parameters_callback(self.validate_limits)
        self.add_post_set_parameters_callback(self.apply_limits)
        self.command_publisher = self.create_publisher(TwistStamped, '/mini_nav/cmd_vel_raw', 1)
        self.lease_publisher = self.create_publisher(Bool, '/mini_nav/task_active', 1)
        status_qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                               durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.status_publisher = self.create_publisher(String, '/mini_nav/ps5/status', status_qos)
        self.enable_service = self.create_service(SetBool, '/mini_nav/ps5/set_enabled', self.set_enabled)
        if controller is None:
            from dualforge import DualForge
            controller = DualForge()
        self.controller = controller
        controller.on_state(self.buffer.connection)
        controller.on_input(self.buffer.receive)
        self.connection_error = None
        self.graph_conflict = False
        self.feedback_enabled = None
        self.rumble_deadline = None
        self.clock_stamp = None
        self.clock_advanced = time.monotonic()
        steady = Clock(clock_type=ClockType.STEADY_TIME)
        self.publish_timer = self.create_timer(0.05, self.tick, clock=steady)
        self.connect_timer = self.create_timer(1.0, self.connect, clock=steady)
        self.get_logger().info(f'USB PS5: left Y drives ({linear} m/s), right X steers ({angular} rad/s); L1 toggles control')

    def validate_limits(self, parameters):
        for parameter in parameters:
            if parameter.name in ['linear_limit', 'angular_limit']:
                if (parameter.type_ != Parameter.Type.DOUBLE or not math.isfinite(parameter.value)
                        or not 0 <= parameter.value <= 2.0):
                    return SetParametersResult(successful=False, reason='Speed limits must be doubles in [0, 2]')
        return SetParametersResult(successful=True)

    def apply_limits(self, parameters):
        if any(parameter.name in ['linear_limit', 'angular_limit'] for parameter in parameters):
            self.buffer.set_limits(self.get_parameter('linear_limit').value,
                                   self.get_parameter('angular_limit').value)

    def set_enabled(self, request, response):
        self.tick()
        response.success, response.message = self.buffer.set_enabled(request.data)
        self.tick()
        if request.data and response.success and not self.buffer.snapshot()[2]:
            response.success = False
            response.message = 'Control was revoked by the watchdog or controller feedback'
        return response

    def connect(self):
        if self.controller.is_connected():
            return
        try:
            self.controller.connect()
            self.controller.stop_rumble()
            self.controller.set_led(255, 0, 0)
            self.feedback_enabled = False
            self.rumble_deadline = None
            self.connection_error = None
            self.get_logger().info('DualSense connected')
        except Exception as error:
            self.buffer.connection(False)
            message = str(error)
            if message != self.connection_error:
                self.get_logger().warning(f'USB controller unavailable: {message}')
                self.connection_error = message
            self.controller.disconnect()

    def tick(self):
        output = self.get_publishers_info_by_topic('/cmd_vel')
        conflict = len(output) != 1 or output[0].node_name != 'velocity_guard'
        for topic in ['/mini_nav/cmd_vel_raw', '/mini_nav/task_active']:
            publishers = self.get_publishers_info_by_topic(topic)
            conflict = conflict or len(publishers) != 1 or publishers[0].node_name != self.get_name()
        if conflict != self.graph_conflict:
            self.graph_conflict = conflict
            if conflict:
                self.get_logger().error('Control publishers conflict or velocity_guard unavailable; releasing motion lease')
        stamp = self.get_clock().now().to_msg()
        now_ns = stamp.sec * 1000000000 + stamp.nanosec
        current = time.monotonic()
        if self.clock_stamp is None or now_ns > self.clock_stamp:
            self.clock_advanced = current
        clock_fault = (now_ns == 0 or (self.clock_stamp is not None and now_ns < self.clock_stamp)
                       or current - self.clock_advanced > 0.35)
        self.clock_stamp = now_ns
        self.buffer.gate(conflict or clock_fault)
        linear, angular, active = self.buffer.snapshot()
        message = TwistStamped()
        message.header.stamp = stamp
        message.header.frame_id = 'base_footprint'
        message.twist.linear.x = linear
        message.twist.angular.z = angular
        self.command_publisher.publish(message)
        self.lease_publisher.publish(Bool(data=active))
        if self.controller.is_connected():
            try:
                if active != self.feedback_enabled:
                    if active:
                        self.controller.set_led(0, 0, 255)
                    else:
                        self.controller.set_led(255, 0, 0)
                    self.controller.set_rumble(100, 100)
                    self.rumble_deadline = current + 0.15
                    self.feedback_enabled = active
                    self.get_logger().info('Control enabled: blue LED' if active else 'Control disabled: red LED')
                if self.rumble_deadline is not None and current >= self.rumble_deadline:
                    self.controller.stop_rumble()
                    self.rumble_deadline = None
            except Exception as error:
                self.buffer.disable()
                self.get_logger().warning(f'Controller feedback failed: {error}', throttle_duration_sec=5.0)
        linear, angular, active = self.buffer.snapshot()
        if conflict:
            reason = 'publisher_conflict'
        elif clock_fault:
            reason = 'clock_fault'
        elif not self.controller.is_connected():
            reason = 'usb_disconnected'
        else:
            with self.buffer.lock:
                fresh = (self.buffer.axes is not None and self.buffer.received is not None
                         and 0 <= current - self.buffer.received <= self.buffer.timeout)
            reason = ('enabled' if active else 'ready') if fresh else 'waiting_input'
        self.status_publisher.publish(String(data=json.dumps({
            'connected': self.controller.is_connected(), 'enabled': active,
            'reason': reason, 'linear_limit': self.get_parameter('linear_limit').value,
            'angular_limit': self.get_parameter('angular_limit').value,
            'linear_command': linear, 'angular_command': angular})))

    def stop(self):
        self.buffer.connection(False)
        self.tick()
        if self.controller.is_connected():
            self.controller.stop_rumble()
            self.controller.set_led(255, 0, 0)
        self.controller.disconnect()


def main():
    stopped = False

    def request_stop(signum, frame):
        nonlocal stopped
        stopped = True

    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    signal.signal(signal.SIGINT, request_stop)
    signal.signal(signal.SIGTERM, request_stop)
    node = Ps5Teleop()
    try:
        while rclpy.ok() and not stopped:
            rclpy.spin_once(node, timeout_sec=0.1)
    except ExternalShutdownException:
        pass
    finally:
        if rclpy.ok():
            node.stop()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
