#!/bin/bash
# CERISE Nav2 Setup Script

# Source ROS2
source /opt/ros/humble/setup.bash
source ~/ros2_workspace/install/setup.bash

# Env vars
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export TURTLEBOT3_MODEL=waffle
export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:~/ros2_workspace/install/nav2_bringup/share/nav2_bringup/worlds

echo "✅ CERISE Nav2 environment loaded"
