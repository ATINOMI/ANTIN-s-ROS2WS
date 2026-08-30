#!/usr/bin/env bash
# Source this file before running the isolated mini-navigation experiment.

source /opt/ros/jazzy/setup.bash

if [ -f /home/a/ros2_ws/install/setup.bash ]; then
  source /home/a/ros2_ws/install/setup.bash
fi

export ROS_DOMAIN_ID=61
export GZ_PARTITION=mini_nav
export TURTLEBOT3_MODEL=burger
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
