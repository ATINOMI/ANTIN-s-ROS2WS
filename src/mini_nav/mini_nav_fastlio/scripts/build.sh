#!/usr/bin/env bash
set -euo pipefail
export CMAKE_BUILD_PARALLEL_LEVEL=2
export MAKEFLAGS=-j2
scurm_root=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd -P)
scurm_ws=$(cd -- "$scurm_root/../../.." && pwd -P)
set +u
source /opt/ros/jazzy/setup.bash
source "$scurm_ws/install_fastlivo/local_setup.bash"
source "$scurm_ws/install/local_setup.bash"
set -u
cd "$scurm_ws"
python3 "$scurm_root/scripts/prepare.py"
python3 "$scurm_root/scripts/configure.py"
colcon --log-base log_scurm/build build --build-base build_mini_nav_fastlio --install-base install_mini_nav_fastlio \
  --base-paths "$scurm_root/../mini_nav_nodes/src/localization/fastlio2/fast_lio" \
  "$scurm_root/../mini_nav_nodes/src/localization/fastlio2/icp_relocalization" \
  "$scurm_root/upstream/SCURM_SentryNavigation/nav2_plugins/costmap_intensity" \
  "$scurm_root/upstream/SCURM_SentryNavigation/autonomous_exploration_development_environment/src/terrain_analysis" \
  --executor sequential --cmake-args -DCMAKE_BUILD_TYPE=Release -DCMAKE_CXX_STANDARD=17 -DBUILD_TESTING=OFF \
  --event-handlers console_cohesion+
set +u
source "$scurm_ws/install_mini_nav_fastlio/local_setup.bash"
set -u
colcon --log-base log_scurm/build build --build-base build_mini_nav_fastlio --install-base install_mini_nav_fastlio \
  --base-paths "$scurm_root/scurm_sim" --cmake-args -DBUILD_TESTING=ON \
  --event-handlers console_cohesion+
