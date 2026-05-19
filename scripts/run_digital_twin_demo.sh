#!/bin/bash
# Coleta /detection_error e gera gráfico.
# Pré-requisito: T1 (stack), T2 (goals) e T3 (yolo_detector) já rodando.
# NÃO sobe goals nem detector — apenas coleta e plota.

DURATION=${1:-300}   # segundos de coleta (padrão 5 min)
WS="$HOME/Documentos/Projetos/cerise-turtlebot3-nav"
ERROR_LOG="/tmp/detection_error_$(date +%Y%m%d_%H%M%S).txt"

source /opt/ros/humble/setup.bash
source "$WS/install/local_setup.bash"

echo "=============================="
echo " CERISE Digital Twin — Coleta"
echo " Duração: ${DURATION}s"
echo " Log: $ERROR_LOG"
echo "=============================="
echo ""
echo "Coletando /detection_error... (Ctrl+C para parar antes)"
echo ""

echo "# detection_error (metros) — $(date)" > "$ERROR_LOG"
ros2 topic echo /detection_error --field data >> "$ERROR_LOG" &
ECHO_PID=$!

trap 'echo ""; echo "Interrompido."' INT
sleep "$DURATION" || true
trap - INT

kill $ECHO_PID 2>/dev/null
wait 2>/dev/null

echo "Log salvo em: $ERROR_LOG"
echo ""
echo "Gerando gráfico..."
python3 "$WS/scripts/plot_detection_error.py" "$ERROR_LOG"
