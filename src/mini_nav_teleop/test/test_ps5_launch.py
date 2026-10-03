import importlib.util
import os
from pathlib import Path

import pytest
import rclpy
from rclpy.context import Context
from rclpy.signals import SignalHandlerOptions
from geometry_msgs.msg import TwistStamped

root = Path(__file__).resolve().parents[1]
spec = importlib.util.spec_from_file_location('ps5_launch', root / 'launch/ps5_teleop.launch.py')
module = importlib.util.module_from_spec(spec)
spec.loader.exec_module(module)


def test_preflight_uses_its_context_and_rejects_existing_controllers():
    assert os.environ.get('ROS_DOMAIN_ID') == '225', 'Preflight test requires isolated domain 225'
    assert module.check_controllers(None) == []
    context = Context()
    rclpy.init(context=context, signal_handler_options=SignalHandlerOptions.NO)
    node = rclpy.create_node('existing_control_probe', context=context)
    publisher = node.create_publisher(TwistStamped, '/cmd_vel', 1)
    try:
        with pytest.raises(RuntimeError, match='Another controller'):
            module.check_controllers(None)
        assert context.ok()
    finally:
        node.destroy_publisher(publisher)
        node.destroy_node()
        rclpy.shutdown(context=context)
