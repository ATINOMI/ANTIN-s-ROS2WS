"""Simulated USB only: exercise the real driver parser and ROS control node."""
import importlib.util
import json
import os
import signal
import sys
from pathlib import Path

if os.environ.get('ROS_DOMAIN_ID') != '222':
    raise RuntimeError('Fixture requires isolated domain 222')
root = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(root / 'vendor'))
from dualforge import DualForge
import rclpy
from rclpy.signals import SignalHandlerOptions
from std_srvs.srv import SetBool
from std_msgs.msg import String
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy

spec = importlib.util.spec_from_file_location('ps5_teleop', root / 'scripts/ps5_teleop.py')
module = importlib.util.module_from_spec(spec)
spec.loader.exec_module(module)


class SimulatedUsb:
    def __init__(self):
        self.driver = DualForge()
        self.connected = False
        self.available = True
        self.l1 = False
        self.led = None
        self.pulses = 0

    def on_input(self, callback): self.driver.on_input(callback)
    def on_state(self, callback): self.driver.on_state(callback)
    def is_connected(self): return self.connected
    def connect(self):
        if not self.available:
            raise RuntimeError('Simulated USB disconnected')
        self.connected = True
        self.driver._on_state(True)
    def disconnect(self):
        self.connected = False
        self.driver._on_state(False)
    def set_led(self, *rgb): self.led = rgb
    def set_rumble(self, *motors): self.pulses += 1
    def stop_rumble(self): pass
    def feed(self):
        if self.connected:
            report = bytearray(63)
            report[0:4] = bytes([128, 0, 255, 128])
            report[7] = 8
            report[8] = int(self.l1)
            self.driver._on_raw_input(report)


rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
stopped = False
def stop(signum, frame):
    global stopped
    stopped = True
signal.signal(signal.SIGINT, stop)
signal.signal(signal.SIGTERM, stop)
controller = SimulatedUsb()
node = module.Ps5Teleop(controller)
qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                 durability=DurabilityPolicy.TRANSIENT_LOCAL)
feedback = node.create_publisher(String, '/ps5_test/feedback', qos)

def sample():
    controller.feed()
    feedback.publish(String(data=json.dumps({'led': controller.led, 'pulses': controller.pulses})))
timer = node.create_timer(0.01, sample)

def usb(request, response):
    controller.available = request.data
    if request.data:
        controller.connect()
    else:
        controller.disconnect()
    response.success = True
    return response

def l1(request, response):
    controller.l1 = request.data
    response.success = True
    return response

services = [node.create_service(SetBool, '/ps5_test/usb_connected', usb),
            node.create_service(SetBool, '/ps5_test/l1', l1)]
try:
    while rclpy.ok() and not stopped:
        rclpy.spin_once(node, timeout_sec=0.02)
finally:
    node.stop()
    node.destroy_node()
    rclpy.shutdown()
