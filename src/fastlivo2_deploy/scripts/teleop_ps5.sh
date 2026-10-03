#!/usr/bin/env bash
set -eo pipefail
task_root=$(cd "$(dirname "${BASH_SOURCE[0]}")/../../.." && pwd)
source /opt/ros/jazzy/setup.bash
source "$task_root/install/setup.bash"
source "$task_root/install_fastlivo/setup.bash"
export ROS_DOMAIN_ID=219
export GZ_PARTITION=mini_nav_fastlivo_deploy
export ROS_LOG_DIR="$task_root/log_fastlivo/runtime"
export PYTHONPATH="$task_root/install_fastlivo_deps/python:${PYTHONPATH:-}"
mkdir -p "$ROS_LOG_DIR"
exec 9>"$task_root/log_fastlivo/teleop_ps5.lock"
flock -n 9 || { echo 'PS5 teleop is already running'; exit 1; }
python3 - <<'PY'
import time
import rclpy
from rclpy.node import Node
rclpy.init()
node = Node('ps5_control_preflight')
deadline = time.monotonic() + 2.0
while time.monotonic() < deadline:
    rclpy.spin_once(node, timeout_sec=0.1)
conflicts = [topic for topic in ['/cmd_vel', '/mini_nav/cmd_vel_raw', '/mini_nav/task_active']
             if node.get_publishers_info_by_topic(topic)]
clock = node.get_publishers_info_by_topic('/clock')
node.destroy_node()
rclpy.shutdown()
if conflicts:
    raise SystemExit(f'Another controller is active on {conflicts}; stop it before manual control')
if not clock:
    raise SystemExit('No simulation clock in domain 219; start run_sim.sh first')
PY
exec ros2 launch mini_nav_teleop ps5_teleop.launch.py "$@"
