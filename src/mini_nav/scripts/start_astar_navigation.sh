#!/usr/bin/env bash

set -eo pipefail

script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
workspace_dir="$(cd "${script_dir}/../../.." && pwd)"
rviz_config="${script_dir}/../rviz/astar_navigation.rviz"
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

ros2 run mini_nav_nodes costmap_publisher_node &
node_pid=$!

cleanup() {
  if kill -0 "${node_pid}" 2>/dev/null; then
    kill "${node_pid}"
    wait "${node_pid}" 2>/dev/null || true
  fi
}
trap cleanup EXIT

rviz2 -d "${rviz_config}"
