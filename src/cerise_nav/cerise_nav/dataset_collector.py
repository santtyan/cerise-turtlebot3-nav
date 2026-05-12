"""
Dataset collector: grava imagens + anotações YOLO via callbacks individuais.
Simples e robusto — sem ApproximateTimeSynchronizer.

Com teleport (collect_teleport.py), a pose fica estável enquanto a imagem é capturada.
"""

import os
import time

import cv2
import rclpy
import rclpy.qos
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from nav_msgs.msg import Odometry
from cv_bridge import CvBridge

from cerise_nav.projection import world_to_pixel_with_camera, robot_bbox_normalized


CAMERA_HEIGHT  = 3.0
ROBOT_RADIUS   = 0.17
ROBOT_CLASS_ID = 0
_PROJECT_DIR   = '/home/yan/Documentos/Projetos/cerise-turtlebot3-nav'
OUT_IMAGES     = f'{_PROJECT_DIR}/dataset/raw/images'
OUT_LABELS     = f'{_PROJECT_DIR}/dataset/raw/annotations'
SAVE_INTERVAL  = 0.5


class DatasetCollector(Node):
    def __init__(self):
        super().__init__('dataset_collector')
        self.bridge = CvBridge()
        self.camera_info = None
        self.poses = {}
        self.last_save = 0.0

        os.makedirs(OUT_IMAGES, exist_ok=True)
        os.makedirs(OUT_LABELS, exist_ok=True)

        existing = [f for f in os.listdir(OUT_IMAGES) if f.endswith('.jpg')]
        self.frame_id = len(existing)

        sensor_qos = rclpy.qos.QoSPresetProfiles.SENSOR_DATA.value
        self.create_subscription(CameraInfo, '/camera/camera_info', self._camera_info_cb, sensor_qos)
        self.create_subscription(Image, '/camera/image_raw', self._image_cb, sensor_qos)
        self.create_subscription(Odometry, '/robot1/odom', self._odom1_cb, 10)
        self.create_subscription(Odometry, '/robot2/odom', self._odom2_cb, 10)

        self.get_logger().info(f'DatasetCollector iniciado | saida={OUT_IMAGES}')
        self._img_count = 0

    def _camera_info_cb(self, msg: CameraInfo):
        self.camera_info = msg

    def _odom1_cb(self, msg: Odometry):
        self.poses['robot1'] = (msg.pose.pose.position.x, msg.pose.pose.position.y)

    def _odom2_cb(self, msg: Odometry):
        self.poses['robot2'] = (msg.pose.pose.position.x, msg.pose.pose.position.y)

    def _image_cb(self, image_msg: Image):
        self._img_count += 1
        if self._img_count % 50 == 1:
            self.get_logger().info(
                f'Imagens recebidas: {self._img_count} | poses={list(self.poses.keys())} | camera_info={"OK" if self.camera_info else "AGUARDANDO"}'
            )
        now = time.time()
        if now - self.last_save < SAVE_INTERVAL:
            return
        if self.camera_info is None:
            return
        if len(self.poses) < 2:
            return

        frame = self.bridge.imgmsg_to_cv2(image_msg, 'bgr8')
        w_norm, h_norm = robot_bbox_normalized(CAMERA_HEIGHT, self.camera_info, ROBOT_RADIUS)

        lines = []
        visible = []
        for name in ['robot1', 'robot2']:
            if name not in self.poses:
                continue
            wx, wy = self.poses[name]
            result = world_to_pixel_with_camera(wx, wy, CAMERA_HEIGHT, self.camera_info)
            if result is None:
                continue
            cx, cy = result
            lines.append(f'{ROBOT_CLASS_ID} {cx:.6f} {cy:.6f} {w_norm:.6f} {h_norm:.6f}')
            visible.append(f'{name}@({wx:.2f},{wy:.2f})')

        if not lines:
            return

        cv2.imwrite(f'{OUT_IMAGES}/{self.frame_id:06d}.jpg', frame)
        with open(f'{OUT_LABELS}/{self.frame_id:06d}.txt', 'w') as f:
            f.write('\n'.join(lines) + '\n')

        self.get_logger().info(f'Frame {self.frame_id:06d} | {visible}')
        self.frame_id += 1
        self.last_save = now


def main():
    rclpy.init()
    node = DatasetCollector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
