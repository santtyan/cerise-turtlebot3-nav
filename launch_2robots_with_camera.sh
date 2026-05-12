#!/bin/bash
# Lançamento 2 TurtleBots + câmera overhead + Nav2 para coleta YOLO dataset

set -e

export DISPLAY=:0
export QT_QPA_PLATFORM=xcb
export TURTLEBOT3_MODEL=waffle
export ROS_DOMAIN_ID=0
export GAZEBO_MODEL_DATABASE_URI=""

source /opt/ros/humble/setup.bash
source "$(dirname "${BASH_SOURCE[0]}")/install/local_setup.bash" 2>/dev/null || true

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
MAP=/opt/ros/humble/share/nav2_bringup/maps/turtlebot3_world.yaml
WORLD=$SCRIPT_DIR/world_with_camera.world
WAFFLE=/opt/ros/humble/share/nav2_bringup/worlds/waffle.model
# URDF sem xacro — frames sem prefixo (base_link, base_footprint, etc.)
URDF=/opt/ros/humble/share/nav2_bringup/urdf/turtlebot3_waffle.urdf
# params locais com use_sim_time: False (evita bloqueio do lifecycle_manager por /clock QoS mismatch)
P1=$SCRIPT_DIR/params_r1.yaml
P2=$SCRIPT_DIR/params_r2.yaml

# ── 1. Gazebo ──────────────────────────────────────────────────────────────
echo "[1/5] gzserver..."
gzserver --verbose -s libgazebo_ros_init.so -s libgazebo_ros_factory.so -e ode "$WORLD" \
  > /tmp/gz.log 2>&1 &

echo -n "Aguardando gzserver"
for i in {1..60}; do
  ros2 service list 2>/dev/null | grep -q '/spawn_entity' && { echo " [OK]"; break; }
  echo -n "."; sleep 1
done

# ── 2. Spawn ───────────────────────────────────────────────────────────────
echo "[2/5] Spawn robot1 e robot2..."
ros2 run gazebo_ros spawn_entity.py \
  -entity robot1 -file "$WAFFLE" -robot_namespace robot1 \
  -x 0.0 -y 0.5 -z 0.01 2>&1 | grep -i status
ros2 run gazebo_ros spawn_entity.py \
  -entity robot2 -file "$WAFFLE" -robot_namespace robot2 \
  -x 0.0 -y -0.5 -z 0.01 2>&1 | grep -i status

# ── 3. Robot State Publisher ───────────────────────────────────────────────
echo "[3/5] robot_state_publisher..."
ROBOT_DESC="$(cat $URDF)"

ros2 run robot_state_publisher robot_state_publisher \
  --ros-args \
  -r __ns:=/robot1 \
  -r /tf:=tf \
  -r /tf_static:=tf_static \
  -p use_sim_time:=false \
  -p robot_description:="$ROBOT_DESC" \
  > /tmp/rsp1.log 2>&1 &

ros2 run robot_state_publisher robot_state_publisher \
  --ros-args \
  -r __ns:=/robot2 \
  -r /tf:=tf \
  -r /tf_static:=tf_static \
  -p use_sim_time:=false \
  -p robot_description:="$ROBOT_DESC" \
  > /tmp/rsp2.log 2>&1 &

# ── 4. Nav2 (inclui map_server + AMCL + planner) ──────────────────────────
echo "[4/5] Nav2 robot1..."
ros2 launch nav2_bringup bringup_launch.py \
  use_sim_time:=False \
  namespace:=robot1 \
  use_namespace:=True \
  autostart:=True \
  map:=$MAP \
  params_file:=$P1 \
  > /tmp/nav2_r1.log 2>&1 &

echo "[5/5] Nav2 robot2..."
ros2 launch nav2_bringup bringup_launch.py \
  use_sim_time:=False \
  namespace:=robot2 \
  use_namespace:=True \
  autostart:=True \
  map:=$MAP \
  params_file:=$P2 \
  > /tmp/nav2_r2.log 2>&1 &

# ── 5. Aguardar Nav2 + publicar TF estatico map→odom ─────────────────────────
echo ""
echo "Aguardando Nav2 ativar (60s)..."
sleep 60

echo "[OK] Verificando se Nav2 ativou..."
ros2 service list 2>/dev/null | grep navigate_to_pose && echo "[OK] navigate_to_pose disponivel!" || echo "[WARN] navigate_to_pose NAO encontrado — verifique /tmp/nav2_r1.log"

echo "[OK] Publicando TF estatico map→odom (bypass AMCL)..."
ros2 run tf2_ros static_transform_publisher \
  --x 0 --y 0 --z 0 --yaw 0 --pitch 0 --roll 0 \
  --frame-id map --child-frame-id odom \
  --ros-args -r __ns:=/robot1 -r /tf_static:=tf_static \
  > /tmp/tf_r1.log 2>&1 &
ros2 run tf2_ros static_transform_publisher \
  --x 0 --y 0 --z 0 --yaw 0 --pitch 0 --roll 0 \
  --frame-id map --child-frame-id odom \
  --ros-args -r __ns:=/robot2 -r /tf_static:=tf_static \
  > /tmp/tf_r2.log 2>&1 &

echo ""
echo "═══════════════════════════════════════════"
echo " [OK] Nav2 pronto! Abra os outros terminais:"
echo "  T2: ros2 run cerise_nav demand_generator"
echo "  T3: ros2 run cerise_nav task_allocator"
echo "  T4: ros2 run cerise_nav dataset_collector"
echo "  T5: gzclient  (GUI opcional)"
echo "═══════════════════════════════════════════"

wait
