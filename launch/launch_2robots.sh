#!/bin/bash
# Lançamento robusto de simulação 2 TurtleBots
# Fix: gzserver primeiro → aguarda /spawn_entity → spawn → Nav2

export DISPLAY=""
export QT_QPA_PLATFORM=offscreen
export TURTLEBOT3_MODEL=waffle
export ROS_DOMAIN_ID=0
export GAZEBO_MODEL_DATABASE_URI=""   # evita download lento de modelos

source /opt/ros/humble/setup.bash
cd ~/cerise-turtlebot3-nav

MAP=/opt/ros/humble/share/nav2_bringup/maps/turtlebot3_world.yaml
WORLD=/opt/ros/humble/share/nav2_bringup/worlds/world_only.model
WAFFLE=/opt/ros/humble/share/nav2_bringup/worlds/waffle.model
P1=/opt/ros/humble/share/nav2_bringup/params/nav2_multirobot_params_1.yaml
P2=/opt/ros/humble/share/nav2_bringup/params/nav2_multirobot_params_2.yaml
URDF=/opt/ros/humble/share/nav2_bringup/urdf/turtlebot3_waffle.urdf

echo "[1/4] gzserver headless..."
gzserver --verbose \
  -s libgazebo_ros_init.so \
  -s libgazebo_ros_factory.so \
  "$WORLD" > /tmp/gz.log 2>&1 &
GZPID=$!

# Aguarda /spawn_entity (até 120s)
for i in $(seq 1 120); do
  ros2 service list 2>/dev/null | grep -q '/spawn_entity' && { echo "[OK] gzserver pronto em ${i}s"; break; }
  sleep 1
done

echo "[2/4] Spawning robots..."
ros2 run gazebo_ros spawn_entity.py \
  -entity robot1 -file "$WAFFLE" -robot_namespace robot1 \
  -x 0.0 -y 0.5 -z 0.01 2>&1 | grep -i status
ros2 run gazebo_ros spawn_entity.py \
  -entity robot2 -file "$WAFFLE" -robot_namespace robot2 \
  -x 0.0 -y -0.5 -z 0.01 2>&1 | grep -i status

echo "[3/4] Lançando Nav2 (robot_state_publisher + bringup)..."

# RSP com remappings corretos (TF local por namespace)
ROBOT_DESC=$(cat "$URDF")
ros2 launch nav2_bringup tb3_simulation_launch.py \
  namespace:=robot1 use_namespace:=True \
  use_simulator:=False use_rviz:=False \
  map:="$MAP" params_file:="$P1" autostart:=true \
  x_pose:=0.0 y_pose:=0.5 \
  > /tmp/nav1.log 2>&1 &

ros2 launch nav2_bringup tb3_simulation_launch.py \
  namespace:=robot2 use_namespace:=True \
  use_simulator:=False use_rviz:=False \
  map:="$MAP" params_file:="$P2" autostart:=true \
  x_pose:=0.0 y_pose:=-0.5 \
  > /tmp/nav2.log 2>&1 &

echo "[4/4] Aguardando Nav2 (60s)..."
for i in $(seq 1 60); do
  ros2 action list 2>/dev/null | grep -q '/robot1/navigate_to_pose' && { echo "[OK] Nav2 pronto em ${i}s"; break; }
  sleep 1
done

echo "=== Simulação rodando. Ctrl+C para encerrar. ==="
echo "Execute: ./set_initialposes.sh"
echo "Depois:  ros2 action send_goal /robot1/navigate_to_pose nav2_msgs/action/NavigateToPose \"{pose: {header: {frame_id: 'map'}, pose: {position: {x: 1.0, y: 1.0, z: 0.0}, orientation: {w: 1.0}}}}\""

wait $GZPID
