#!/usr/bin/env bash
# 独立文件式后端；不向系统安装 ROS1 或 GTSAM。
set -euo pipefail
root=$(cd "$(dirname "${BASH_SOURCE[0]}")/../../../.." && pwd)
tool="$root/src/mini_nav/mini_nav_fastlivo/offline"
deps="$root/third_party/mini_nav_mapping"
mkdir -p "$deps"
touch "$deps/COLCON_IGNORE"
fetch_fixed() {
    local name="$1" url="$2" revision="$3"
    if ! test -d "$deps/$name/.git"; then
        git init "$deps/$name"
        git -C "$deps/$name" remote add origin "$url"
        git -C "$deps/$name" fetch --depth 1 origin "$revision"
        git -C "$deps/$name" checkout --detach FETCH_HEAD
    fi
    test "$(git -C "$deps/$name" rev-parse HEAD)" = "$revision"
}
fetch_fixed HBA https://github.com/hku-mars/HBA.git a0cdd474996fd9bb76888c8d1839b49b70aa0818
fetch_fixed gtsam https://github.com/borglab/gtsam.git 4f66a491ffc83cf092d0d818b11dc35135521612
cmake -S "$deps/gtsam" -B "$root/build_mapping_deps/gtsam" \
    -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX="$root/install_mapping_deps" \
    -DGTSAM_BUILD_TESTS=OFF -DGTSAM_BUILD_EXAMPLES_ALWAYS=OFF -DGTSAM_BUILD_UNSTABLE=OFF \
    -DGTSAM_BUILD_PYTHON=OFF -DGTSAM_USE_SYSTEM_EIGEN=ON -DGTSAM_WITH_TBB=OFF \
    -DGTSAM_BUILD_WITH_MARCH_NATIVE=OFF
cmake --build "$root/build_mapping_deps/gtsam" -j4
cmake --install "$root/build_mapping_deps/gtsam"
python3 "$tool/prepare_hba.py" "$deps/HBA" "$root/build_mapping_offline/adapted_hba"
cmake -S "$tool" -B "$root/build_mapping_offline/bin" -DCMAKE_BUILD_TYPE=Release \
    -DCMAKE_PREFIX_PATH="$root/install_mapping_deps" \
    -DHBA_ADAPTED_DIR="$root/build_mapping_offline/adapted_hba"
cmake --build "$root/build_mapping_offline/bin" -j2
