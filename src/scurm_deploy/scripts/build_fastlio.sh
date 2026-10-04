#!/usr/bin/env bash
set -euo pipefail
export CMAKE_BUILD_PARALLEL_LEVEL=2
export MAKEFLAGS=-j2
scurm_root=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)
scurm_ws=$(cd -- "$scurm_root/../.." && pwd)
set +u
source /opt/ros/jazzy/setup.bash
source "$scurm_ws/install_fastlivo/local_setup.bash"
source "$scurm_ws/install/local_setup.bash"
set -u
cd "$scurm_ws"
python3 "$scurm_root/scripts/prepare.py" --mapping-only
python3 "$scurm_root/scripts/configure.py"
colcon --log-base log_scurm/fastlio_build build --build-base build_scurm --install-base install_scurm \
  --base-paths "$scurm_root/upstream/SCURM_SentryNavigation/FAST_LIO" "$scurm_root/scurm_sim" \
  --executor sequential --cmake-args -DCMAKE_BUILD_TYPE=Release -DCMAKE_CXX_STANDARD=17 -DBUILD_TESTING=ON \
  --event-handlers console_cohesion+
