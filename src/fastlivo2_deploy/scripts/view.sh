#!/usr/bin/env bash
set -eo pipefail
deploy_root=$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)
task_root=$(cd "$deploy_root/../.." && pwd)
source /opt/ros/jazzy/setup.bash
source "$task_root/install/setup.bash"
source "$task_root/install_fastlivo/setup.bash"
export ROS_DOMAIN_ID=219
export GZ_PARTITION=mini_nav_fastlivo_deploy
export ROS_LOG_DIR="$task_root/log_fastlivo/runtime"
export QT_QPA_PLATFORM="${QT_QPA_PLATFORM:-xcb}"
mkdir -p "$ROS_LOG_DIR"
exec rviz2 -d "$deploy_root/fastlivo_sim/config/localization.rviz" \
  --ros-args -p use_sim_time:=true "$@"
