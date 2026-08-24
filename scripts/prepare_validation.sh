#!/bin/bash
# Prepara o ambiente para uma rodada de validação no Gazebo.
# Uso: bash scripts/prepare_validation.sh [baseline|ppo]
#
# Renomeia o cerise_log.csv existente com timestamp para não perder dados,
# e mata processos zumbis do ROS/Gazebo antes de subir o pipeline.

set -e
POLICY=${1:-"baseline"}
LOG=~/cerise_log.csv
BACKUP=~/cerise_log_$(date +%Y%m%d_%H%M%S)_${POLICY}.csv

# 1. Faz backup do CSV anterior se existir
if [ -f "$LOG" ]; then
    mv "$LOG" "$BACKUP"
    echo "[OK] CSV anterior salvo em $BACKUP"
else
    echo "[OK] Nenhum CSV anterior encontrado"
fi

# 2. Mata processos zumbis do ROS/Gazebo (lição aprendida da sessão 2026-05-18)
echo "[...] Matando processos anteriores do ROS/Gazebo..."
pkill -9 -f "gzserver" 2>/dev/null || true
pkill -9 -f "gzclient" 2>/dev/null || true
pkill -9 -f "component_container_isolated" 2>/dev/null || true
pkill -9 -f "nav2" 2>/dev/null || true
pkill -9 -f "task_allocator" 2>/dev/null || true
pkill -9 -f "rl_task_allocator" 2>/dev/null || true
pkill -9 -f "demand_generator" 2>/dev/null || true
pkill -9 -f "yolo_detector" 2>/dev/null || true
sleep 2
echo "[OK] Processos anteriores encerrados"

echo ""
echo "═══════════════════════════════════════════════════"
echo " Pronto para rodada: $POLICY"
echo ""
echo " Passo 1 (T1): bash launch/launch_2robots_with_camera.sh"
echo " Passo 2:      aguardar 70s até Nav2 ativar"
echo " Passo 3 (T2): ros2 run cerise_nav demand_generator"
if [ "$POLICY" = "ppo" ]; then
echo " Passo 4 (T3): ros2 run cerise_nav rl_task_allocator \\"
echo "                 --ros-args -p model_path:=\$PWD/models/ppo_allocator_yolo.zip"
else
echo " Passo 4 (T3): ros2 run cerise_nav task_allocator"
fi
echo " Passo 5 (T4): ros2 run cerise_nav yolo_detector"
echo " Passo 6:      aguardar ~1h (mínimo 60 demandas)"
echo "═══════════════════════════════════════════════════"
