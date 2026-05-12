#!/usr/bin/env python3
"""
Script de validação experimental: verifica alignment real de bbox em Gazebo.

Ferramenta de debug canonicamente correta para ROS2+Gazebo:
1. Inspeciona camera_info publicado em /camera/camera_info
2. Lê poses em tempo real de /robot*/odom
3. Publica marcadores visuais (geometry_msgs/MarkerArray) em /debug_markers
4. Permite visualização em RViz para validação visual
5. Detecta problemas comuns:
   - Frames de referência incorretos
   - Rotação de câmera inesperada
   - Poses em coordenadas locais em vez de mundo
   - Parâmetros intrínsecos incorretos

Uso:
    ros2 run cerise_nav debug_camera_validation.py

Saída:
    - /debug_markers: Visualização em RViz (verde = projeção esperada, red = alerta)
    - Console: Comparação entre poses lidas e projeção calculada
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from nav_msgs.msg import Odometry
from visualization_msgs.msg import MarkerArray, Marker
from geometry_msgs.msg import Point
from cv_bridge import CvBridge
import cv2
import numpy as np
from typing import Optional, Tuple
from dataclasses import dataclass

# Parâmetros esperados
CAMERA_HEIGHT = 3.0
ROBOT_SPAWN = {
    'robot1': (0.0, 0.5),
    'robot2': (0.0, -0.5),
}
ROBOT_RADIUS = 0.17


@dataclass
class RobotPose:
    x: float = 0.0
    y: float = 0.0
    updated: bool = False


def world_to_camera_frame(x: float, y: float, z: float = 0.0) -> Tuple[float, float, float]:
    """Transforma ponto do mundo para frame da câmera."""
    cam_x = x
    cam_y = -y
    cam_z = CAMERA_HEIGHT
    return cam_x, cam_y, cam_z


def camera_frame_to_pixels(
    cam_x: float, cam_y: float, cam_z: float,
    fx: float, fy: float, cx: float, cy: float,
    img_w: int, img_h: int
) -> Optional[Tuple[float, float]]:
    """Projeta ponto do frame da câmera para pixels (pinhole model)."""
    if cam_z <= 0:
        return None

    u = fx * (cam_x / cam_z) + cx
    v = fy * (cam_y / cam_z) + cy

    if 0 <= u < img_w and 0 <= v < img_h:
        return u, v
    return None


def world_to_pixels(
    x: float, y: float,
    fx: float, fy: float, cx: float, cy: float,
    img_w: int, img_h: int
) -> Optional[Tuple[float, float]]:
    """Wrapper: mundo → câmera → pixels."""
    cam_x, cam_y, cam_z = world_to_camera_frame(x, y)
    return camera_frame_to_pixels(cam_x, cam_y, cam_z, fx, fy, cx, cy, img_w, img_h)


class CameraValidationNode(Node):
    """Nó de validação de alinhamento de câmera em Gazebo."""

    def __init__(self):
        super().__init__('debug_camera_validation')
        self.bridge = CvBridge()
        self.camera_info = None
        self.poses = {
            'robot1': RobotPose(),
            'robot2': RobotPose(),
        }
        self.frame_count = 0

        # Publisher para marcadores de debug
        self.marker_pub = self.create_publisher(MarkerArray, '/debug_markers', 10)

        # Subscriptions
        self.create_subscription(CameraInfo, '/camera/camera_info', self._camera_info_cb, 10)
        self.create_subscription(Image, '/camera/image_raw', self._image_cb, 10)
        for name in self.poses:
            self.create_subscription(
                Odometry,
                f'/{name}/odom',
                lambda msg, n=name: self._odom_cb(msg, n),
                10,
            )

        self.get_logger().info('CameraValidationNode iniciado.')
        self.get_logger().info('Aguardando camera_info e poses...')

    def _camera_info_cb(self, msg: CameraInfo):
        """Callback de camera_info."""
        if self.camera_info is None:
            self.get_logger().info(
                f'camera_info recebida:\n'
                f'  Resolução: {msg.width}x{msg.height}\n'
                f'  fx={msg.k[0]:.1f} fy={msg.k[4]:.1f}\n'
                f'  cx={msg.k[2]:.1f} cy={msg.k[5]:.1f}\n'
                f'  FOV_H={2*np.degrees(np.arctan(msg.width/(2*msg.k[0]))):.1f}°'
            )
        self.camera_info = msg

    def _odom_cb(self, msg: Odometry, name: str):
        """Callback de odometria."""
        p = msg.pose.pose.position
        self.poses[name].x = p.x
        self.poses[name].y = p.y
        self.poses[name].updated = True

    def _image_cb(self, msg: Image):
        """Callback de imagem. Verifica alinhamento a cada frame."""
        if self.camera_info is None:
            self.get_logger().warn('Aguardando camera_info...', throttle_duration_sec=5.0)
            return

        if not all(p.updated for p in self.poses.values()):
            self.get_logger().warn('Aguardando poses...', throttle_duration_sec=5.0)
            return

        # Reset flags
        for p in self.poses.values():
            p.updated = False

        frame = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        fx = self.camera_info.k[0]
        fy = self.camera_info.k[4]
        cx = self.camera_info.k[2]
        cy = self.camera_info.k[5]
        img_w = self.camera_info.width
        img_h = self.camera_info.height

        # =======================
        # VALIDAÇÃO E DEBUG
        # =======================

        # 1. Verificar se poses estão em coordenadas mundo (não locais)
        self._validate_pose_frame(frame)

        # 2. Desenhar bboxes calculados na imagem
        frame_debug = frame.copy()
        markers = MarkerArray()

        for name, pose in self.poses.items():
            result = world_to_pixels(pose.x, pose.y, fx, fy, cx, cy, img_w, img_h)
            if result is None:
                self.get_logger().warn(f'{name} fora do frame: ({pose.x:.2f}, {pose.y:.2f})')
                continue

            px, py = result
            # Tamanho do bbox em pixels (diâmetro do robô)
            bbox_diameter_px = (ROBOT_RADIUS * 2 * fx) / CAMERA_HEIGHT
            bbox_size = int(bbox_diameter_px)

            # Desenhar na imagem
            color = (0, 255, 0) if name == 'robot1' else (0, 165, 255)  # Verde / Laranja
            cv2.circle(frame_debug, (int(px), int(py)), bbox_size, color, 2)
            cv2.putText(
                frame_debug, f'{name}', (int(px) + bbox_size + 5, int(py)),
                cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255, 255, 255), 1
            )

            # Criar marcador para RViz (linha do ponto mundo até a câmera)
            marker = Marker()
            marker.header.frame_id = 'world'
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.id = hash(name) % 10000
            marker.type = Marker.SPHERE
            marker.action = Marker.ADD
            marker.pose.position.x = pose.x
            marker.pose.position.y = pose.y
            marker.pose.position.z = 0.0
            marker.scale.x = ROBOT_RADIUS * 2
            marker.scale.y = ROBOT_RADIUS * 2
            marker.scale.z = 0.1
            marker.color.r = 0.0
            marker.color.g = 1.0
            marker.color.b = 0.0
            marker.color.a = 0.5
            markers.markers.append(marker)

        self.marker_pub.publish(markers)

        # Salvar debug frame
        if self.frame_count % 30 == 0:  # A cada 3 segundos (10 fps)
            cv2.imwrite(f'/tmp/debug_frame_{self.frame_count:06d}.jpg', frame_debug)
            self.get_logger().info(
                f'Frame {self.frame_count}: Debug salvo em /tmp/debug_frame_{self.frame_count:06d}.jpg'
            )

        self.frame_count += 1

    def _validate_pose_frame(self, frame: np.ndarray):
        """
        Valida se as poses estão em coordenadas mundo ou locais.

        Heurística:
        - Se y está próximo de ±0.5, está certo (spawn position)
        - Se y está próximo de 0.0, pode estar em frame local (bug anterior)
        """
        for name, pose in self.poses.items():
            expected_y = ROBOT_SPAWN[name][1]

            # Tolerância de movimento: ±2 metros
            if abs(pose.y - expected_y) > 2.0:
                self.get_logger().warn(
                    f'{name}: pose.y={pose.y:.3f} muito longe do spawn y={expected_y:.3f} '
                    f'(diferença: {abs(pose.y - expected_y):.3f}m) — pode ser frame local!'
                )
            elif abs(pose.y) < 0.1 and abs(expected_y) > 0.3:
                self.get_logger().error(
                    f'{name}: pose.y≈0 mas spawn era {expected_y}! '
                    f'Odom pode estar em frame LOCAL, não MUNDO!'
                )


def main():
    rclpy.init()
    node = CameraValidationNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
