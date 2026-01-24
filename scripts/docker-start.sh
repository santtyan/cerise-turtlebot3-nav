#!/bin/bash

# Script para iniciar o container Docker ROS2

set -e

WORKSPACE_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

echo "🐳 Iniciando container ROS2 Humble..."

# Verificar se o Docker está rodando
if ! docker info > /dev/null 2>&1; then
    echo "❌ Docker não está rodando. Por favor, inicie o Docker primeiro."
    exit 1
fi

# Verificar se o container já existe
if docker ps -a | grep -q ros2-humble; then
    echo "📦 Container existente encontrado"
    
    # Verificar se está rodando
    if docker ps | grep -q ros2-humble; then
        echo "✅ Container já está rodando"
        echo "🔌 Conectando ao container..."
        docker exec -it ros2-humble bash
    else
        echo "▶️  Iniciando container..."
        docker start ros2-humble
        docker exec -it ros2-humble bash
    fi
else
    echo "🆕 Criando novo container..."
    
    # Criar o container usando docker run
    docker run -it \
        --name ros2-humble \
        -e DISPLAY=$DISPLAY \
        -v /tmp/.X11-unix:/tmp/.X11-unix \
        -v "$WORKSPACE_DIR:/root/ros2_workspace" \
        osrf/ros:humble-desktop-full \
        bash -c "source /opt/ros/humble/setup.bash && cd /root/ros2_workspace && exec bash"
fi
