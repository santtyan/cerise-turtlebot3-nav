"""
Coleta dataset teleportando robôs para posições diversas via Gazebo.
Captura imagem + gera anotação YOLO diretamente (não depende de dataset_collector).

Estratégia: combos × yaws × jitter → ~300 frames únicos com poses muito variadas.
"""
import os
import math
import time
import random
import cv2
import rclpy
import rclpy.qos
from rclpy.node import Node
from gazebo_msgs.srv import SetEntityState
from gazebo_msgs.msg import EntityState
from geometry_msgs.msg import Pose
from sensor_msgs.msg import Image, CameraInfo
from nav_msgs.msg import Odometry
from cv_bridge import CvBridge

import sys
sys.path.insert(0, '/home/yan/Documentos/Projetos/cerise-turtlebot3-nav/src/cerise_nav')
from cerise_nav.projection import world_to_pixel_with_camera, robot_bbox_normalized

random.seed(42)

# 30 combos base cobrindo workspace [-1.8, 1.8]
_BASE_COMBOS = [
    ((-1.6, -1.6), ( 1.6,  1.6)), ((-1.6,  0.0), ( 1.6,  0.0)),
    ((-1.6,  1.6), ( 1.6, -1.6)), (( 0.0, -1.6), ( 0.0,  1.6)),
    (( 1.6, -1.6), (-1.6,  1.6)), (( 1.6,  0.0), (-1.6,  0.0)),
    (( 1.6,  1.6), (-1.6, -1.6)), (( 0.0,  1.6), ( 0.0, -1.6)),
    ((-1.0, -1.0), ( 1.0,  1.0)), ((-1.0,  1.0), ( 1.0, -1.0)),
    (( 1.0,  1.0), (-1.0, -1.0)), (( 1.0, -1.0), (-1.0,  1.0)),
    ((-0.6,  0.0), ( 0.6,  0.0)), (( 0.0, -0.6), ( 0.0,  0.6)),
    ((-1.4,  0.8), ( 1.4, -0.8)), (( 1.4,  0.8), (-1.4, -0.8)),
    ((-0.8,  1.4), ( 0.8, -1.4)), (( 0.8,  1.4), (-0.8, -1.4)),
    ((-1.2, -0.4), ( 1.2,  0.4)), (( 1.2, -0.4), (-1.2,  0.4)),
    ((-0.4, -1.2), ( 0.4,  1.2)), (( 0.4, -1.2), (-0.4,  1.2)),
    ((-1.6,  0.8), ( 0.8,  1.6)), (( 1.6, -0.8), (-0.8, -1.6)),
    ((-0.8,  1.6), ( 1.6, -0.8)), (( 0.8, -1.6), (-1.6,  0.8)),
    ((-1.4, -1.4), ( 0.0,  0.0)), (( 0.0,  0.0), ( 1.4,  1.4)),
    (( 1.4, -1.4), ( 0.0,  0.0)), (( 0.0,  0.0), (-1.4,  1.4)),
]

# 4 yaws para cada combo → diversidade direcional
_YAWS = [0.0, math.pi / 2, math.pi, -math.pi / 2]

# Build full list: 30 combos × 4 yaws + jitter ±10cm = 120 frames base
# Repete 3x com jitter diferente → ~360 frames brutos, filtra colisões ~300 reais
_RUNS = []
for run in range(3):
    for (r1, r2) in _BASE_COMBOS:
        for yaw_offset in _YAWS:
            jx1 = random.uniform(-0.1, 0.1)
            jy1 = random.uniform(-0.1, 0.1)
            jx2 = random.uniform(-0.1, 0.1)
            jy2 = random.uniform(-0.1, 0.1)
            _RUNS.append((
                (r1[0] + jx1, r1[1] + jy1),
                (r2[0] + jx2, r2[1] + jy2),
                yaw_offset,
            ))

random.shuffle(_RUNS)
POSITIONS_R1 = [r[0] for r in _RUNS]
POSITIONS_R2 = [r[1] for r in _RUNS]
YAWS_LIST = [r[2] for r in _RUNS]

PROJECT_DIR  = '/home/yan/Documentos/Projetos/cerise-turtlebot3-nav'
OUT_IMAGES   = f'{PROJECT_DIR}/dataset/raw/images'
OUT_LABELS   = f'{PROJECT_DIR}/dataset/raw/annotations'
CAMERA_H     = 3.0
ROBOT_RADIUS = 0.17
CLASS_ID     = 0


class TeleportCollector(Node):
    def __init__(self):
        super().__init__('teleport_collector')
        self.bridge = CvBridge()
        self.camera_info = None
        self.latest_image = None

        self.odom_poses = {}
        sensor_qos = rclpy.qos.QoSPresetProfiles.SENSOR_DATA.value
        self.create_subscription(CameraInfo, '/camera/camera_info', self._info_cb, sensor_qos)
        self.create_subscription(Image, '/camera/image_raw', self._img_cb, sensor_qos)
        self.create_subscription(Odometry, '/robot1/odom', lambda m: self._odom_cb('robot1', m), 10)
        self.create_subscription(Odometry, '/robot2/odom', lambda m: self._odom_cb('robot2', m), 10)

        self.cli = self.create_client(SetEntityState, '/set_entity_state')
        self.get_logger().info('Aguardando serviço Gazebo...')
        self.cli.wait_for_service(timeout_sec=15.0)
        self.get_logger().info('Pronto!')

        os.makedirs(OUT_IMAGES, exist_ok=True)
        os.makedirs(OUT_LABELS, exist_ok=True)

        existing = [f for f in os.listdir(OUT_IMAGES) if f.endswith('.jpg')]
        self.frame_id = len(existing)

    def _info_cb(self, msg):
        self.camera_info = msg

    def _img_cb(self, msg):
        self.latest_image = msg

    def _odom_cb(self, name, msg):
        self.odom_poses[name] = (msg.pose.pose.position.x, msg.pose.pose.position.y)

    def teleport(self, name, x, y, yaw=0.0):
        req = SetEntityState.Request()
        state = EntityState()
        state.name = name
        state.pose = Pose()
        state.pose.position.x = x
        state.pose.position.y = y
        state.pose.position.z = 0.01
        state.pose.orientation.z = math.sin(yaw / 2)
        state.pose.orientation.w = math.cos(yaw / 2)
        state.reference_frame = 'world'
        req.state = state
        future = self.cli.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=2.0)

    def wait_for_image(self, timeout=3.0):
        deadline = time.time() + timeout
        self.latest_image = None
        while time.time() < deadline:
            rclpy.spin_once(self, timeout_sec=0.1)
            if self.latest_image is not None:
                return self.latest_image
        return None

    def wait_for_camera_info(self, timeout=10.0):
        deadline = time.time() + timeout
        while time.time() < deadline:
            rclpy.spin_once(self, timeout_sec=0.1)
            if self.camera_info is not None:
                return True
        return False

    def flush_image_queue(self, duration=1.0):
        """Descarta imagens antigas da fila para garantir imagem fresca."""
        deadline = time.time() + duration
        while time.time() < deadline:
            rclpy.spin_once(self, timeout_sec=0.05)
        self.latest_image = None

    def capture(self, poses):
        if self.camera_info is None:
            self.get_logger().warning('camera_info ausente, pulando frame')
            return

        # Flush fila antiga, depois aguarda imagem fresca após teleport
        self.flush_image_queue(duration=1.0)
        img_msg = self.wait_for_image(timeout=3.0)
        if img_msg is None:
            self.get_logger().warning('Timeout esperando imagem, pulando frame')
            return

        frame = self.bridge.imgmsg_to_cv2(img_msg, 'bgr8')
        w_norm, h_norm = robot_bbox_normalized(CAMERA_H, self.camera_info, ROBOT_RADIUS)

        # Usa odom real (onde o robô realmente está) em vez das coords planejadas
        actual_poses = {}
        for name in poses:
            if name in self.odom_poses:
                actual_poses[name] = self.odom_poses[name]
            else:
                actual_poses[name] = poses[name]

        lines = []
        for name, (wx, wy) in actual_poses.items():
            result = world_to_pixel_with_camera(wx, wy, CAMERA_H, self.camera_info)
            if result is None:
                continue
            cx, cy = result
            lines.append(f'{CLASS_ID} {cx:.6f} {cy:.6f} {w_norm:.6f} {h_norm:.6f}')

        if not lines:
            self.get_logger().warning('Nenhum robô visível, pulando frame')
            return

        cv2.imwrite(f'{OUT_IMAGES}/{self.frame_id:06d}.jpg', frame)
        with open(f'{OUT_LABELS}/{self.frame_id:06d}.txt', 'w') as f:
            f.write('\n'.join(lines) + '\n')

        self.get_logger().info(f'Frame {self.frame_id:06d} salvo | poses={list(poses.keys())}')
        self.frame_id += 1

    def run(self):
        self.get_logger().info('Aguardando camera_info...')
        if not self.wait_for_camera_info():
            self.get_logger().error('camera_info não recebida! Abortando.')
            return

        combos = list(zip(POSITIONS_R1, POSITIONS_R2, YAWS_LIST))
        self.get_logger().info(f'{len(combos)} combinações de poses (combos × yaws × jitter)')

        for i, ((x1, y1), (x2, y2), yaw_offset) in enumerate(combos):
            if abs(x1 - x2) < 0.3 and abs(y1 - y2) < 0.3:
                continue
            yaw1 = math.atan2(y2 - y1, x2 - x1) + yaw_offset
            yaw2 = math.atan2(y1 - y2, x1 - x2) + yaw_offset
            self.teleport('robot1', x1, y1, yaw1)
            self.teleport('robot2', x2, y2, yaw2)
            time.sleep(0.8)  # aguarda física estabilizar

            poses = {'robot1': (x1, y1), 'robot2': (x2, y2)}
            self.capture(poses)
            if (i + 1) % 20 == 0:
                self.get_logger().info(f'[{i+1}/{len(combos)}] coletados {self.frame_id} frames')

        self.get_logger().info(f'Coleta concluida! {self.frame_id} frames salvos em {OUT_IMAGES}')


def main():
    rclpy.init()
    node = TeleportCollector()
    node.run()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
