#!/bin/bash
# Lançamento 3 TurtleBots + câmera overhead + Nav2 para validação RL

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
URDF=/opt/ros/humble/share/nav2_bringup/urdf/turtlebot3_waffle.urdf
P1=$SCRIPT_DIR/params_r1.yaml
P2=$SCRIPT_DIR/params_r2.yaml
P3=$SCRIPT_DIR/params_r3.yaml

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
# Spawns distribuídos nos 3 quadrantes livres da arena.
# Regra: evitar a grade de cilindros {-1.1,0,1.1}^2 (raio ~0.15m).
# Posições escolhidas a ≥0.5m de qualquer cilindro e entre os waypoints A-D,
# reduzindo cruzamentos. Todos dentro do FOV da câmera (±1.73m H, ±1.3m V).
echo "[2/5] Spawn robot1, robot2 e robot3 (distribuídos nos quadrantes)..."
ros2 run gazebo_ros spawn_entity.py \
  -entity robot1 -file "$WAFFLE" -robot_namespace robot1 \
  -x  0.80 -y  0.40 -z 0.01 2>&1 | grep -i status
ros2 run gazebo_ros spawn_entity.py \
  -entity robot2 -file "$WAFFLE" -robot_namespace robot2 \
  -x -0.80 -y  0.40 -z 0.01 2>&1 | grep -i status
ros2 run gazebo_ros spawn_entity.py \
  -entity robot3 -file "$WAFFLE" -robot_namespace robot3 \
  -x  0.00 -y -0.70 -z 0.01 2>&1 | grep -i status

# ── 3. Robot State Publisher ───────────────────────────────────────────────
echo "[3/5] robot_state_publisher..."
ROBOT_DESC="$(cat $URDF)"

ros2 run robot_state_publisher robot_state_publisher \
  --ros-args \
  -r __ns:=/robot1 -r /tf:=tf -r /tf_static:=tf_static \
  -p use_sim_time:=false -p robot_description:="$ROBOT_DESC" \
  > /tmp/rsp1.log 2>&1 &

ros2 run robot_state_publisher robot_state_publisher \
  --ros-args \
  -r __ns:=/robot2 -r /tf:=tf -r /tf_static:=tf_static \
  -p use_sim_time:=false -p robot_description:="$ROBOT_DESC" \
  > /tmp/rsp2.log 2>&1 &

ros2 run robot_state_publisher robot_state_publisher \
  --ros-args \
  -r __ns:=/robot3 -r /tf:=tf -r /tf_static:=tf_static \
  -p use_sim_time:=false -p robot_description:="$ROBOT_DESC" \
  > /tmp/rsp3.log 2>&1 &

# ── 4. Nav2 ────────────────────────────────────────────────────────────────
echo "[4/5] Nav2 robot1..."
ros2 launch nav2_bringup bringup_launch.py \
  use_sim_time:=False namespace:=robot1 use_namespace:=True \
  autostart:=True map:=$MAP params_file:=$P1 \
  > /tmp/nav2_r1.log 2>&1 &

echo "      Nav2 robot2..."
ros2 launch nav2_bringup bringup_launch.py \
  use_sim_time:=False namespace:=robot2 use_namespace:=True \
  autostart:=True map:=$MAP params_file:=$P2 \
  > /tmp/nav2_r2.log 2>&1 &

echo "      Nav2 robot3..."
ros2 launch nav2_bringup bringup_launch.py \
  use_sim_time:=False namespace:=robot3 use_namespace:=True \
  autostart:=True map:=$MAP params_file:=$P3 \
  > /tmp/nav2_r3.log 2>&1 &

# ── 5. Aguardar + TF estático map→odom ────────────────────────────────────
echo ""
echo "Aguardando Nav2 ativar (70s)..."
sleep 70

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

ros2 run tf2_ros static_transform_publisher \
  --x 0 --y 0 --z 0 --yaw 0 --pitch 0 --roll 0 \
  --frame-id map --child-frame-id odom \
  --ros-args -r __ns:=/robot3 -r /tf_static:=tf_static \
  > /tmp/tf_r3.log 2>&1 &

echo ""
echo "═══════════════════════════════════════════════════════"
echo " [OK] Nav2 pronto! Abra os outros terminais:"
echo ""
echo "  T2: ros2 run cerise_nav demand_generator"
echo ""
echo "  T3 (baseline):  ros2 run cerise_nav task_allocator"
echo "  T3 (PPO):       ros2 run cerise_nav rl_task_allocator \\"
echo "                    --ros-args -p model_path:=\$PWD/models/ppo_allocator_yolo.zip \\"
echo "                    -p robots:='[\"robot1\",\"robot2\",\"robot3\"]'"
echo ""
echo "  T4: ros2 run cerise_nav yolo_detector"
echo "  T5: gzclient  (GUI opcional)"
echo "═══════════════════════════════════════════════════════"

wait
