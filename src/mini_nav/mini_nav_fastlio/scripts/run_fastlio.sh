#!/usr/bin/env bash
set -euo pipefail
scurm_root=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd -P)
scurm_ws=$(cd -- "$scurm_root/../../.." && pwd -P)
export ROS_DOMAIN_ID=231
export GZ_PARTITION=scurm_fastlio2_demo
export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export TURTLEBOT3_MODEL=waffle
export ROS_LOG_DIR="$scurm_ws/log_scurm/fastlio_runtime"
mkdir -p "$ROS_LOG_DIR"
exec 9>"$ROS_LOG_DIR/launch.lock"
flock -n 9 || { echo 'FAST-LIO2 demo is already running'; exit 1; }
echo "$$" > "$ROS_LOG_DIR/launch.pid"
set +u
source /opt/ros/jazzy/setup.bash
source "$scurm_ws/install_fastlivo/local_setup.bash"
source "$scurm_ws/install/local_setup.bash"
source "$scurm_ws/install_mini_nav_fastlio/local_setup.bash"
set -u
exec ros2 launch scurm_sim fastlio_mapping.launch.py "$@"
