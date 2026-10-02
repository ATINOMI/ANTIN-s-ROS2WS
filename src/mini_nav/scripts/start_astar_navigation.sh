#!/usr/bin/env bash

set -eo pipefail

script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
workspace_dir="$(cd "${script_dir}/../../.." && pwd)"
rviz_config="${script_dir}/../rviz/astar_navigation.rviz"
map_yaml="${script_dir}/../mini_nav_bringup/maps/tb3_learning.yaml"
ros_setup="/opt/ros/jazzy/setup.bash"

if [[ ! -f "${ros_setup}" ]]; then
  echo "ROS 2 Jazzy setup file was not found: ${ros_setup}" >&2
  exit 1
fi

source "${ros_setup}"

cd "${workspace_dir}"

source "${workspace_dir}/install/setup.bash"

if [[ ! -f "${rviz_config}" ]]; then
  echo "RViz configuration was not found: ${rviz_config}" >&2
  exit 1
fi

if [[ ! -f "${map_yaml}" ]]; then
  echo "Map YAML was not found: ${map_yaml}" >&2
  exit 1
fi

existing_publishers="$(
  { timeout 5 ros2 topic info --no-daemon --spin-time 2 /mini_nav/map 2>/dev/null |
      awk '/Publisher count:/ { print $3; exit }'; } || true
)"
if [[ "${existing_publishers:-0}" != "0" ]]; then
  echo "Existing /mini_nav/map publishers detected; stop them before starting this script." >&2
  exit 1
fi

setsid ros2 run mini_nav_nodes costmap_publisher_node --ros-args \
  -p "map_file:=${map_yaml}" -p planning.use_initial_pose_as_start:=true &
node_pid=$!
rviz_runtime_config=""

cleanup() {
  if [[ -n "${rviz_runtime_config}" && -f "${rviz_runtime_config}" ]]; then
    rm -f "${rviz_runtime_config}"
  fi
  if kill -0 "${node_pid}" 2>/dev/null; then
    kill -- "-${node_pid}" 2>/dev/null || kill "${node_pid}"
    wait "${node_pid}" 2>/dev/null || true
  fi
}
trap cleanup EXIT

# 等待首张地图，使用实际发布的几何信息配置 RViz 视角。
map_info="$(timeout 10 ros2 topic echo --no-daemon --spin-time 3 --qos-reliability reliable --qos-durability transient_local --once /mini_nav/map --field info)" || {
  echo "Timed out while waiting for /mini_nav/map" >&2
  exit 1
}
resolution="$(awk '/^resolution:/ { print $2; exit }' <<< "${map_info}")"
width="$(awk '/^width:/ { print $2; exit }' <<< "${map_info}")"
height="$(awk '/^height:/ { print $2; exit }' <<< "${map_info}")"
origin_x="$(awk '/^origin:/ { origin = 1; next } origin && /^  position:/ { position = 1; next } position && /^    x:/ { print $2; exit }' <<< "${map_info}")"
origin_y="$(awk '/^origin:/ { origin = 1; next } origin && /^  position:/ { position = 1; next } position && /^    y:/ { print $2; exit }' <<< "${map_info}")"

if [[ -z "${resolution}" || -z "${width}" || -z "${height}" ||
      -z "${origin_x}" || -z "${origin_y}" ]]; then
  echo "Could not read map geometry from /mini_nav/map" >&2
  exit 1
fi

read -r rviz_distance focal_x focal_y < <(
  awk -v resolution="${resolution}" -v width="${width}" -v height="${height}" \
      -v origin_x="${origin_x}" -v origin_y="${origin_y}" '
    BEGIN {
      map_width = width * resolution
      map_height = height * resolution
      if (map_width <= 0.0 || map_height <= 0.0) {
        exit 1
      }
      max_extent = map_width > map_height ? map_width : map_height
      # 保持默认 Orbit 视角与地图最大边的相对视野不变。
      distance = max_extent * 0.402549147605896
      printf "%.15g %.15g %.15g\n", distance,
             origin_x + map_width / 2.0, origin_y + map_height / 2.0
    }')

if [[ -z "${rviz_distance}" || -z "${focal_x}" || -z "${focal_y}" ]]; then
  echo "Could not calculate RViz view from map geometry" >&2
  exit 1
fi

rviz_runtime_config="$(mktemp /tmp/mini_nav_rviz_XXXXXX)"
sed -e "s/^      Distance:.*/      Distance: ${rviz_distance}/" \
    -e "/Focal Point:/,/Focal Shape/ s/^        X:.*/        X: ${focal_x}/" \
    -e "/Focal Point:/,/Focal Shape/ s/^        Y:.*/        Y: ${focal_y}/" \
    "${rviz_config}" > "${rviz_runtime_config}"

rviz2 -d "${rviz_runtime_config}"
