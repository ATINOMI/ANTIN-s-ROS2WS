#!/usr/bin/env bash
set -euo pipefail
scurm_root=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd -P)
scurm_ws=$(cd -- "$scurm_root/../../.." && pwd -P)
export ROS_DOMAIN_ID=231
export GZ_PARTITION=scurm_jazzy_gazebo
export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
export TURTLEBOT3_MODEL=waffle
export ROS_LOG_DIR="$scurm_ws/log_scurm/runtime"
mkdir -p "$ROS_LOG_DIR"
exec 9>"$scurm_ws/log_scurm/scurm.lock"
flock -n 9 || { echo 'SCURM simulation is already running'; exit 1; }
set +u
source /opt/ros/jazzy/setup.bash
source "$scurm_ws/install_fastlivo/local_setup.bash"
source "$scurm_ws/install/local_setup.bash"
source "$scurm_ws/install_mini_nav_fastlio/local_setup.bash"
set -u
exec ros2 launch scurm_sim bringup.launch.py \
  map:="$scurm_ws/src/mini_nav/mini_nav_bringup/maps/tb3_learning.yaml" "$@"
