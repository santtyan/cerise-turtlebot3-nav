"""
Dataset collector: grava imagens + anotações YOLO para treino de detecção de robôs.

Uma única classe ('robot') é usada para todos os robôs, pois YOLO não consegue
distinguir objetos fisicamente idênticos. A identificação robot1/robot2 pode ser
feita posteriormente via tracking temporal ou coloração diferenciada dos modelos.

Subscreve:
  /camera/image_raw       -> frames JPG
  /camera/camera_info     -> parâmetros intrínsecos para projeção correta
  /robot1/odom            -> pose robot1 (não requer Nav2/AMCL)
  /robot2/odom            -> pose robot2 (não requer Nav2/AMCL)

Grava em:
  dataset/raw/images/NNN.jpg
  dataset/raw/annotations/NNN.txt  (formato YOLO: class cx cy w h normalizados)

Use scripts/split_dataset.py para separar em train/val após a coleta.
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
from cv_bridge import CvBridge
import cv2, os, time
from dataclasses import dataclass
from cerise_nav.projection import world_to_pixel_with_camera, robot_bbox_normalized


CAMERA_HEIGHT  = 3.0   # metros — deve bater com world_with_camera.model <pose>
ROBOT_RADIUS   = 0.17  # metros — TurtleBot3 Waffle
ROBOT_CLASS_ID = 0     # uma classe única: 'robot'
OUT_IMAGES     = 'dataset/raw/images'
OUT_LABELS     = 'dataset/raw/annotations'


@dataclass
class RobotPose:
    x: float = 0.0
    y: float = 0.0
    updated: bool = False


class DatasetCollector(Node):
    def __init__(self):
        super().__init__('dataset_collector')
        self.bridge = CvBridge()
        self.camera_info = None
        self.poses = {
            'robot1': RobotPose(),
            'robot2': RobotPose(),
        }
        self.frame_id = 0
        self.save_interval = 1.0

        os.makedirs(OUT_IMAGES, exist_ok=True)
        os.makedirs(OUT_LABELS, exist_ok=True)

        self.create_subscription(CameraInfo, '/camera/camera_info', self._camera_info_cb, 10)
        self.create_subscription(Image, '/camera/image_raw', self._image_cb, 10)
        for name in self.poses:
            self.create_subscription(
                Odometry,
                f'/{name}/odom',
                lambda msg, n=name: self._odom_cb(msg, n),
                10,
            )
        self.last_save = 0.0
        self.get_logger().info(
            f'DatasetCollector iniciado. Saida: {OUT_IMAGES}, {OUT_LABELS}'
        )

    def _camera_info_cb(self, msg: CameraInfo):
        self.camera_info = msg

    def _odom_cb(self, msg: Odometry, name: str):
        p = msg.pose.pose.position
        self.poses[name].x = p.x
        self.poses[name].y = p.y
        self.poses[name].updated = True

    def _pose_cb(self, msg: PoseWithCovarianceStamped, name: str):
        p = msg.pose.pose.position
        self.poses[name].x = p.x
        self.poses[name].y = p.y
        self.poses[name].updated = True

    def _image_cb(self, msg: Image):
        now = time.time()
        if now - self.last_save < self.save_interval:
            return
        if self.camera_info is None:
            self.get_logger().warn('Aguardando /camera/camera_info...', throttle_duration_sec=5.0)
            return
        if not all(p.updated for p in self.poses.values()):
            self.get_logger().warn('Aguardando poses de todos os robos...', throttle_duration_sec=5.0)
            return
        for p in self.poses.values():
            p.updated = False

        frame = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        w_norm, h_norm = robot_bbox_normalized(CAMERA_HEIGHT, self.camera_info, ROBOT_RADIUS)

        # Coleta anotações apenas dos robôs visíveis (não descarta o frame inteiro)
        lines = []
        visible_names = []
        for name, pose in self.poses.items():
            result = world_to_pixel_with_camera(pose.x, pose.y, CAMERA_HEIGHT, self.camera_info)
            if result is None:
                continue
            cx, cy = result
            lines.append(f'{ROBOT_CLASS_ID} {cx:.6f} {cy:.6f} {w_norm:.6f} {h_norm:.6f}')
            visible_names.append(name)

        if not lines:
            self.get_logger().warn('Nenhum robo visivel no frame — ignorado')
            return

        img_path = f'{OUT_IMAGES}/{self.frame_id:06d}.jpg'
        ann_path = f'{OUT_LABELS}/{self.frame_id:06d}.txt'
        cv2.imwrite(img_path, frame)
        with open(ann_path, 'w') as f:
            f.write('\n'.join(lines) + '\n')

        self.get_logger().info(
            f'Frame {self.frame_id:06d} | visiveis={visible_names} | bbox=({w_norm:.3f}x{h_norm:.3f})'
        )
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
