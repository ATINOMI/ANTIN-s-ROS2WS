#!/usr/bin/env bash
set -euo pipefail
scurm_root=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd -P)
scurm_ws=$(cd -- "$scurm_root/../../.." && pwd -P)
export ROS_DOMAIN_ID=231
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
set +u
source /opt/ros/jazzy/setup.bash
set -u
exec python3 "$scurm_root/scripts/save_pcd.py" \
    --output-dir "$scurm_ws/maps/scurm_fastlio2" "$@"
