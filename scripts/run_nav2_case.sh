#!/usr/bin/env bash

# 统一启动 nav2_learning 和 nav2_stvl_demo 导航案例。
# 用法：
#   ./scripts/run_nav2_case.sh learning
#   ./scripts/run_nav2_case.sh stvl

set -e

WORKSPACE="/home/a/ros2_ws"
CASE="${1:-}"

if [[ -z "${CASE}" ]]; then
  echo "请选择要启动的案例："
  echo "  1) learning - nav2_learning 普通导航"
  echo "  2) stvl     - nav2_stvl_demo STVL 导航"
  read -r -p "请输入 1 或 2：" choice

  case "${choice}" in
    1) CASE="learning" ;;
    2) CASE="stvl" ;;
    *) echo "无效选择。"; exit 1 ;;
  esac
fi

source /opt/ros/jazzy/setup.bash
source "${WORKSPACE}/install/setup.bash"

case "${CASE}" in
  learning)
    export ROS_DOMAIN_ID=42
    export GZ_PARTITION=nav2_learning

    echo "启动 nav2_learning 普通导航案例..."
    echo "ROS_DOMAIN_ID=${ROS_DOMAIN_ID}"
    echo "GZ_PARTITION=${GZ_PARTITION}"

    exec ros2 launch nav2_learning localization_nav2.launch.py \
      headless:=False \
      use_rviz:=True
    ;;

  stvl)
    export ROS_DOMAIN_ID=48
    export GZ_PARTITION=nav2_stvl_demo

    echo "启动 nav2_stvl_demo STVL 导航案例..."
    echo "ROS_DOMAIN_ID=${ROS_DOMAIN_ID}"
    echo "GZ_PARTITION=${GZ_PARTITION}"

    exec ros2 launch nav2_stvl_demo stvl_nav2.launch.py \
      headless:=False \
      use_rviz:=True
    ;;

  *)
    echo "未知案例：${CASE}"
    echo "可选参数：learning 或 stvl"
    exit 1
    ;;
esac
