#!/usr/bin/env bash
set -eo pipefail
task_root=$(cd "$(dirname "${BASH_SOURCE[0]}")/../../.." && pwd)
source /opt/ros/jazzy/setup.bash
source "$task_root/install/setup.bash"
source "$task_root/install_fastlivo/setup.bash"
export ROS_DOMAIN_ID=219
export GZ_PARTITION=mini_nav_fastlivo_deploy
export FASTLIVO_MAP_DIR="$task_root/maps/fastlivo2"
export ROS_LOG_DIR="$task_root/log_fastlivo/runtime"
mkdir -p "$ROS_LOG_DIR"
exec 9>"$task_root/log_fastlivo/simulation.lock"
flock -n 9 || { echo 'FAST-LIVO Gazebo deployment is already running'; exit 1; }
exec ros2 launch fastlivo_sim bringup.launch.py "$@"
