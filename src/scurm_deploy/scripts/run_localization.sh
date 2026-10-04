#!/usr/bin/env bash
set -eo pipefail
scurm_root=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)
scurm_ws=$(cd -- "$scurm_root/../.." && pwd)
export ROS_DOMAIN_ID=227
export GZ_PARTITION=scurm_mini_nav
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
export TURTLEBOT3_MODEL=waffle
export QT_QPA_PLATFORM=xcb
mkdir -p "$scurm_ws/log_scurm/localization_runtime"
exec 9>"$scurm_ws/log_scurm/localization_runtime/launch.lock"
flock -n 9 || { echo 'SCURM mini_nav is already running'; exit 1; }
echo $$ > "$scurm_ws/log_scurm/localization_runtime/launch.pid"
source /opt/ros/jazzy/setup.bash
source "$scurm_ws/install_fastlivo/local_setup.bash"
source "$scurm_ws/install/local_setup.bash"
source "$scurm_ws/install_scurm/local_setup.bash"
exec ros2 launch scurm_sim mini_nav_fastlio.launch.py "$@"
