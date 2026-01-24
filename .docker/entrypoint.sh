#!/bin/bash

# Script de entrada automatizado para o container ROS2

echo "🚀 Inicializando ambiente ROS2 Humble..."

# Source do ROS2 Humble
if [ -f /opt/ros/humble/setup.bash ]; then
    source /opt/ros/humble/setup.bash
    echo "✅ ROS2 Humble configurado"
fi

# Source do workspace local (se existir)
if [ -f /root/ros2_workspace/install/setup.bash ]; then
    source /root/ros2_workspace/install/setup.bash
    echo "✅ Workspace local configurado"
fi

# Mudar para o diretório do workspace
cd /root/ros2_workspace

# Verificar se Navigation2 está instalado
if ros2 pkg list | grep -q nav2_bringup; then
    echo "✅ Navigation2 está instalado"
else
    echo "⚠️  Navigation2 não encontrado. Execute: apt install ros-humble-navigation2 ros-humble-nav2-bringup"
fi

echo ""
echo "📁 Workspace: /root/ros2_workspace"
echo "🔧 ROS_DISTRO: $ROS_DISTRO"
echo "📦 Para instalar Navigation2: apt install ros-humble-navigation2 ros-humble-nav2-bringup"
echo "🔨 Para compilar: colcon build"
echo ""

# Executar o comando passado como argumento ou iniciar bash
exec "$@"
