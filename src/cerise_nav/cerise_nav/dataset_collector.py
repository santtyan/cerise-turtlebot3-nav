"""
Dataset collector: grava imagens + posições dos robôs para treino YOLO.

Subscreve:
  /camera/image_raw          → frames JPG
  /robot1/amcl_pose          → pose robot1 no mapa
  /robot2/amcl_pose          → pose robot2 no mapa

Grava em:
  dataset/images/NNN.jpg
  dataset/annotations/NNN.txt  (formato YOLO: class cx cy w h)
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import PoseWithCovarianceStamped
from cv_bridge import CvBridge
import cv2, os, csv, time
from dataclasses import dataclass, field
from typing import Optional


@dataclass
class RobotPose:
    x: float = 0.0
    y: float = 0.0
    updated: bool = False


class DatasetCollector(Node):
    def __init__(self):
        super().__init__('dataset_collector')
        self.bridge = CvBridge()
        self.poses: dict[str, RobotPose] = {
            'robot1': RobotPose(),
            'robot2': RobotPose(),
        }
        self.frame_id = 0
        self.save_interval = 1.0  # segundos entre frames

        os.makedirs('dataset/images', exist_ok=True)
        os.makedirs('dataset/annotations', exist_ok=True)

        self.create_subscription(Image, '/camera/image_raw', self.image_cb, 10)
        for name in self.poses:
            self.create_subscription(
                PoseWithCovarianceStamped,
                f'/{name}/amcl_pose',
                lambda msg, n=name: self.pose_cb(msg, n),
                10
            )
        self.last_save = 0.0
        self.get_logger().info('DatasetCollector iniciado')

    def pose_cb(self, msg: PoseWithCovarianceStamped, name: str):
        p = msg.pose.pose.position
        self.poses[name].x = p.x
        self.poses[name].y = p.y
        self.poses[name].updated = True

    def image_cb(self, msg: Image):
        now = time.time()
        if now - self.last_save < self.save_interval:
            return
        if not all(p.updated for p in self.poses.values()):
            return  # aguarda poses de todos os robôs

        frame = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        h, w = frame.shape[:2]

        img_path = f'dataset/images/{self.frame_id:06d}.jpg'
        ann_path = f'dataset/annotations/{self.frame_id:06d}.txt'

        cv2.imwrite(img_path, frame)

        # Anotação YOLO (posição em pixels normalizada — PLACEHOLDER)
        # TODO: projetar coordenadas mapa→pixel com parâmetros da câmera
        with open(ann_path, 'w') as f:
            for idx, (name, pose) in enumerate(self.poses.items()):
                # cx cy w h normalizados (0-1) — substituir pela projeção real
                cx, cy, bw, bh = 0.5, 0.5, 0.1, 0.1
                f.write(f'0 {cx:.4f} {cy:.4f} {bw:.4f} {bh:.4f}\n')

        self.get_logger().info(f'Frame {self.frame_id} salvo')
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
