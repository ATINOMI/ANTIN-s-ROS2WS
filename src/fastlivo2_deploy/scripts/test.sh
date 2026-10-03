#!/usr/bin/env bash
set -eo pipefail
task_root=$(cd "$(dirname "${BASH_SOURCE[0]}")/../../.." && pwd)
cd "$task_root"
source /opt/ros/jazzy/setup.bash
source install_fastlivo/setup.bash
colcon --log-base log_fastlivo test --base-paths \
  src/fastlivo2_deploy/FAST-LIVO2 \
  src/fastlivo2_deploy/rpg_vikit/vikit_common \
  src/fastlivo2_deploy/rpg_vikit/vikit_ros \
  src/fastlivo2_deploy/livox_ros_driver2 \
  src/fastlivo2_deploy/fastlivo_sim \
  --build-base build_fastlivo --install-base install_fastlivo \
  --event-handlers console_cohesion+
colcon test-result --test-result-base build_fastlivo --verbose
