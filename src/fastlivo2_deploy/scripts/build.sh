#!/usr/bin/env bash
set -eo pipefail
task_root=$(cd "$(dirname "${BASH_SOURCE[0]}")/../../.." && pwd)
cd "$task_root"
source /opt/ros/jazzy/setup.bash
source "$task_root/install/setup.bash"
cmake -S src/fastlivo2_deploy/Sophus -B build_fastlivo_deps/sophus \
  -DCMAKE_INSTALL_PREFIX="$task_root/install_fastlivo_deps" \
  -DBUILD_SOPHUS_TESTS=OFF -DBUILD_PYTHON_BINDINGS=OFF \
  -DCMAKE_EXPORT_PACKAGE_REGISTRY=OFF
cmake --install build_fastlivo_deps/sophus
export CMAKE_PREFIX_PATH="$task_root/install_fastlivo_deps:${CMAKE_PREFIX_PATH:-}"
export CMAKE_BUILD_PARALLEL_LEVEL=2
colcon --log-base log_fastlivo build --base-paths \
  src/fastlivo2_deploy/FAST-LIVO2 \
  src/fastlivo2_deploy/rpg_vikit/vikit_common \
  src/fastlivo2_deploy/rpg_vikit/vikit_ros \
  src/fastlivo2_deploy/livox_ros_driver2 \
  src/fastlivo2_deploy/fastlivo_sim \
  --build-base build_fastlivo --install-base install_fastlivo \
  --executor sequential --packages-up-to fastlivo_sim \
  --cmake-args -DBUILD_TESTING=ON -DBUILD_LIVOX_DRIVER=OFF \
  --event-handlers console_cohesion+
