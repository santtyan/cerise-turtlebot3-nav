#!/bin/bash
# Valida se câmera overhead está sendo publicada corretamente

export DISPLAY=""
export QT_QPA_PLATFORM=offscreen
export TURTLEBOT3_MODEL=waffle
export ROS_DOMAIN_ID=0
export GAZEBO_MODEL_DATABASE_URI=""

source /opt/ros/humble/setup.bash

echo "Lançando gzserver com câmera por 30s..."
gzserver --verbose -s libgazebo_ros_init.so -s libgazebo_ros_factory.so world_with_camera.model > /dev/null 2>&1 &
GZPID=$!
sleep 15

echo "Procurando /camera/image_raw..."
if ros2 topic list 2>/dev/null | grep -q '/camera/image_raw'; then
    echo "✅ Câmera /camera/image_raw disponível"
    echo "Lendo 3 frames..."
    timeout 5 ros2 topic echo /camera/image_raw --once 2>/dev/null | head -10
else
    echo "❌ Câmera /camera/image_raw NÃO ENCONTRADA"
    echo "Tópicos disponíveis:"
    ros2 topic list 2>/dev/null | head -20
fi

kill $GZPID 2>/dev/null
wait $GZPID 2>/dev/null
