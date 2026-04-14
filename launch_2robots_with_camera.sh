#!/bin/bash
# Lançamento 2 TurtleBots + câmera overhead para coleta YOLO dataset

export DISPLAY=:0
export QT_QPA_PLATFORM=xcb
export TURTLEBOT3_MODEL=waffle
export ROS_DOMAIN_ID=0
export GAZEBO_MODEL_DATABASE_URI=""

source /opt/ros/humble/setup.bash

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
MAP=/opt/ros/humble/share/nav2_bringup/maps/turtlebot3_world.yaml
WORLD=$SCRIPT_DIR/world_with_camera.model
WAFFLE=$SCRIPT_DIR/waffle_nodepth.model
P1=/opt/ros/humble/share/nav2_bringup/params/nav2_multirobot_params_1.yaml
P2=/opt/ros/humble/share/nav2_bringup/params/nav2_multirobot_params_2.yaml

echo "[1/4] gzserver + câmera overhead..."
gzserver --verbose -s libgazebo_ros_init.so -s libgazebo_ros_factory.so -e ode "$WORLD" > /tmp/gz_camera.log 2>&1 &
GZPID=$!

for i in {1..120}; do
  ros2 service list 2>/dev/null | grep -q '/spawn_entity' && { echo "[OK] gzserver pronto"; break; }
  sleep 1
done

echo "[2/4] Spawn robots..."
ros2 run gazebo_ros spawn_entity.py -entity robot1 -file "$WAFFLE" -robot_namespace robot1 -x 0.0 -y 0.5 -z 0.01 2>&1 | grep status
ros2 run gazebo_ros spawn_entity.py -entity robot2 -file "$WAFFLE" -robot_namespace robot2 -x 0.0 -y -0.5 -z 0.01 2>&1 | grep status

echo "[3/4] Nav2..."
ros2 launch nav2_bringup tb3_simulation_launch.py namespace:=robot1 use_namespace:=True use_simulator:=False use_rviz:=False map:= params_file:= autostart:=true x_pose:=0.0 y_pose:=0.5 > /tmp/nav1_camera.log 2>&1 &
ros2 launch nav2_bringup tb3_simulation_launch.py namespace:=robot2 use_namespace:=True use_simulator:=False use_rviz:=False map:= params_file:= autostart:=true x_pose:=0.0 y_pose:=-0.5 > /tmp/nav2_camera.log 2>&1 &

for i in {1..90}; do
  ros2 action list 2>/dev/null | grep -q navigate_to_pose && break
  sleep 1
done

echo "[4/4] Pronto"
echo "Terminal 2: ./set_initialposes.sh"
echo "Terminal 3: ros2 run cerise_nav dataset_collector"
echo "Camera: ros2 topic echo /camera/image_raw"

wait 
