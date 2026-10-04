#!/usr/bin/env bash
set -eo pipefail
export CMAKE_BUILD_PARALLEL_LEVEL=2
export MAKEFLAGS=-j2
scurm_root=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd -P)
scurm_ws=$(cd -- "$scurm_root/../../.." && pwd -P)
source /opt/ros/jazzy/setup.bash
source "$scurm_ws/install_fastlivo/local_setup.bash"
cd "$scurm_ws"
colcon --log-base log_scurm/mini_nav_build build \
  --packages-select mini_nav_core mini_nav_nodes mini_nav_bringup \
  --event-handlers console_cohesion+
source "$scurm_ws/install/local_setup.bash"
python3 "$scurm_root/scripts/bind_prior.py" "${1:-$scurm_ws/maps/scurm_fastlio2/map_20261004_190840_437173/map.pcd}" \
  --map "$scurm_ws/src/mini_nav/mini_nav_bringup/maps/tb3_learning.yaml" --output "$scurm_root/scurm_sim/maps"
colcon --log-base log_scurm/localization_build build --build-base build_mini_nav_fastlio --install-base install_mini_nav_fastlio \
  --base-paths "$scurm_root/../mini_nav_nodes/src/localization/fastlio2/fast_lio" \
  "$scurm_root/../mini_nav_nodes/src/localization/fastlio2/icp_relocalization" "$scurm_root/scurm_sim" \
  --executor sequential --cmake-args -DCMAKE_BUILD_TYPE=Release -DCMAKE_CXX_STANDARD=17 -DBUILD_TESTING=ON \
  --event-handlers console_cohesion+
