#!/bin/bash
# Quick start: Gazebo + 2 robots + camera GUI

export TURTLEBOT3_MODEL=waffle
export ROS_DOMAIN_ID=0
export DISPLAY=:0
export QT_QPA_PLATFORM=xcb
export GAZEBO_MODEL_DATABASE_URI=""

cd "$(dirname "$0")"
source /opt/ros/humble/setup.bash

SCRIPT_DIR="$(pwd)"
WORLD="$SCRIPT_DIR/world_with_camera.model"
ROBOT_MODEL="$SCRIPT_DIR/waffle_nodepth.model"

echo "🚀 Iniciando Gazebo com 2 robots..."
echo "Aguarde a janela do Gazebo aparecer..."
echo ""

# Start gzserver + gzclient
gzserver -s libgazebo_ros_init.so -s libgazebo_ros_factory.so -e ode "$WORLD" &
GZPID=$!

sleep 3

# Spawn robots
ros2 run gazebo_ros spawn_entity.py -entity robot1 -file "$ROBOT_MODEL" -robot_namespace robot1 -x 0.0 -y 0.5 -z 0.01 2>&1 | grep -i "status" &
sleep 1
ros2 run gazebo_ros spawn_entity.py -entity robot2 -file "$ROBOT_MODEL" -robot_namespace robot2 -x 0.0 -y -0.5 -z 0.01 2>&1 | grep -i "status" &

echo "✓ Gazebo rodando em PID $GZPID"
echo ""
echo "Próximos passos em outros terminais:"
echo "  Terminal 2: ./set_initialposes.sh"
echo "  Terminal 3: ros2 run cerise_nav dataset_collector"
echo ""
echo "Para parar: killall gzserver"
echo ""

wait $GZPID
