#!/usr/bin/env bash
# 复用已经部署的 FAST-LIVO2 overlay，为双入口安装固定版本配准依赖。
set -e
task_root=$(cd "$(dirname "${BASH_SOURCE[0]}")/../../../.." && pwd)
task_package="$task_root/src/mini_nav/mini_nav_fastlivo"
test -f "$task_root/install_fastlivo/setup.bash" || {
    echo '请先按 src/fastlivo2_deploy/README.md 构建 FAST-LIVO2。' >&2
    exit 1
}
python3 -m pip install --no-deps --only-binary=:all: --require-hashes --upgrade \
    --target "$task_root/install_fastlivo_deps/python" -r "$task_package/requirements.lock"
source /opt/ros/jazzy/setup.bash
if test -f "$task_root/install/setup.bash"; then source "$task_root/install/setup.bash"; fi
source "$task_root/install_fastlivo/setup.bash"
cd "$task_root"
colcon build --packages-select mini_nav_fastlivo mini_nav_nodes mini_nav_bringup \
    --executor sequential --cmake-args "-DSMALL_GICP_PYTHON_ROOT=$task_root/install_fastlivo_deps/python"
