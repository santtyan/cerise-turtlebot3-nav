#!/bin/bash

# Script para compilar o workspace dentro do container

set -e

WORKSPACE_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

echo "🔨 Compilando workspace ROS2..."

# Verificar se o container está rodando
if ! docker ps | grep -q ros2-humble; then
    echo "⚠️  Container não está rodando. Iniciando..."
    docker start ros2-humble
    sleep 2
fi

# Compilar dentro do container
docker exec -it ros2-humble bash -c "
    source /opt/ros/humble/setup.bash
    cd /root/ros2_workspace
    colcon build
    source install/setup.bash
    echo '✅ Compilação concluída!'
"
