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
colcon --log-base log_scurm/fastlio_build build --build-base build_mini_nav_fastlio --install-base install_mini_nav_fastlio \
  --base-paths "$scurm_root/../mini_nav_core" "$scurm_root/../mini_nav_nodes/src/fastlio2/fast_lio" "$scurm_root/scurm_sim" \
  --executor sequential --cmake-args -DMINI_NAV_BUILD_FASTLIO2=ON -DCMAKE_BUILD_TYPE=Release -DCMAKE_CXX_STANDARD=17 -DBUILD_TESTING=ON \
  --event-handlers console_cohesion+
