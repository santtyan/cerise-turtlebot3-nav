"""
Digital twin sync node: camera image → YOLO → world position → compare with odometry.
Fecha o loop do gêmeo digital: câmera vê robôs → posição estimada → erro vs ground truth.
"""

import os
import math

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from sensor_msgs.msg import Image
from nav_msgs.msg import Odometry
from std_msgs.msg import Float32, String
from geometry_msgs.msg import PoseArray, Pose

import cv2
from cv_bridge import CvBridge
import numpy as np

try:
    from ultralytics import YOLO
    YOLO_AVAILABLE = True
except ImportError:
    YOLO_AVAILABLE = False

from cerise_nav.projection import pixel_to_world_simple


CAMERA_HEIGHT = 3.0
HORIZONTAL_FOV = 1.047   # 60° em radianos
IMG_WIDTH = 640
IMG_HEIGHT = 480
CONF_THRESHOLD = 0.5
MODEL_PATH = os.path.join(
    os.path.dirname(__file__), '..', '..', '..', '..', '..', '..', 'model_robot_detector.pt'
)

SENSOR_QOS = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    history=HistoryPolicy.KEEP_LAST,
    depth=1,
)


class YoloDetector(Node):
    def __init__(self):
        super().__init__('yolo_detector')

        # Parâmetros
        self.declare_parameter('model_path', MODEL_PATH)
        self.declare_parameter('conf_threshold', CONF_THRESHOLD)
        self.declare_parameter('camera_height', CAMERA_HEIGHT)
        self.declare_parameter('horizontal_fov', HORIZONTAL_FOV)

        model_path = self.get_parameter('model_path').value
        self.conf = self.get_parameter('conf_threshold').value
        self.cam_height = self.get_parameter('camera_height').value
        self.fov = self.get_parameter('horizontal_fov').value

        if not YOLO_AVAILABLE:
            self.get_logger().error('ultralytics não instalado. pip install ultralytics')
            raise RuntimeError('ultralytics required')

        # Resolve path relativo ao workspace se não for absoluto
        if not os.path.isabs(model_path):
            ws = os.environ.get('COLCON_PREFIX_PATH', '').split(':')[0]
            model_path = os.path.join(ws, '..', model_path)
        model_path = os.path.realpath(model_path)

        if not os.path.exists(model_path):
            # Fallback: busca no diretório do projeto
            candidates = [
                '/home/yan/Documentos/Projetos/cerise-turtlebot3-nav/model_robot_detector.pt',
                os.path.join(os.path.expanduser('~'), 'model_robot_detector.pt'),
            ]
            for c in candidates:
                if os.path.exists(c):
                    model_path = c
                    break
            else:
                self.get_logger().error(f'Modelo não encontrado: {model_path}')
                raise FileNotFoundError(f'model_robot_detector.pt not found')

        self.get_logger().info(f'Carregando modelo: {model_path}')
        self.model = YOLO(model_path)

        self.bridge = CvBridge()

        # Ground truth: posições dos robôs via odometry
        self.odom = {}  # robot_id → (x, y)

        # Lista de robôs (paridade com task_allocator/rl_task_allocator).
        # Default = 3 robôs para validação; parametrizável para evitar perder GT
        # de algum robô (bug histórico: robot3 sem subscription de odom).
        self.declare_parameter('robots', ['robot1', 'robot2', 'robot3'])
        self.robots = list(self.get_parameter('robots').value)

        # Subscriptions
        self.create_subscription(Image, '/camera/image_raw', self._img_cb, SENSOR_QOS)
        for r in self.robots:
            self.create_subscription(
                Odometry, f'/{r}/odom',
                lambda m, rid=r: self._odom_cb(rid, m), SENSOR_QOS)

        # Publishers
        self.pub_poses = self.create_publisher(PoseArray, '/robot_detections', 10)
        self.pub_error = self.create_publisher(Float32, '/detection_error', 10)
        self.pub_debug = self.create_publisher(String, '/detection_debug', 10)
        self.pub_image = self.create_publisher(Image, '/detection_image', 10)

        self.get_logger().info('YoloDetector pronto — aguardando imagens')

    def _odom_cb(self, robot_id: str, msg: Odometry):
        self.odom[robot_id] = (msg.pose.pose.position.x, msg.pose.pose.position.y)

    def _img_cb(self, msg: Image):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().warn(f'cv_bridge erro: {e}')
            return

        h, w = frame.shape[:2]
        results = self.model.predict(frame, conf=self.conf, verbose=False)

        detections = []
        if results and len(results[0].boxes):
            for box in results[0].boxes:
                # Centro do bbox em coordenadas normalizadas
                cx_px, cy_px = box.xywhn[0][:2].tolist()  # normalizado [0,1]
                raw_x, raw_y = pixel_to_world_simple(
                    cx_px, cy_px,
                    camera_height=self.cam_height,
                    horizontal_fov=self.fov,
                    img_width=w,
                    img_height=h,
                )
                # Câmera pitch=π/2: world_x=raw_y, world_y=-raw_x (validado empiricamente)
                world_x, world_y = raw_y, -raw_x
                conf_val = float(box.conf[0])
                detections.append((world_x, world_y, conf_val))

        self._publish_detections(detections)
        self._compute_error(detections)
        self._publish_image(frame, results)

    def _publish_image(self, frame, results):
        annotated = results[0].plot() if results else frame
        # Overlay: erro médio no canto superior esquerdo
        cv2.putText(annotated, 'CERISE Digital Twin', (10, 25),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
        try:
            msg = self.bridge.cv2_to_imgmsg(annotated, encoding='bgr8')
            msg.header.stamp = self.get_clock().now().to_msg()
            self.pub_image.publish(msg)
        except Exception:
            pass

    def _publish_detections(self, detections):
        msg = PoseArray()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'map'
        for wx, wy, _ in detections:
            p = Pose()
            p.position.x = wx
            p.position.y = wy
            p.position.z = 0.0
            msg.poses.append(p)
        self.pub_poses.publish(msg)

    def _compute_error(self, detections):
        if not detections or not self.odom:
            return

        gt_positions = list(self.odom.values())  # [(x,y), ...]
        det_positions = [(wx, wy) for wx, wy, _ in detections]

        # Associação greedy: each detection to nearest ground truth
        total_error = 0.0
        matched = 0
        used_gt = set()

        for dx, dy in det_positions:
            best_dist = float('inf')
            best_idx = -1
            for i, (gx, gy) in enumerate(gt_positions):
                if i in used_gt:
                    continue
                dist = math.sqrt((dx - gx) ** 2 + (dy - gy) ** 2)
                if dist < best_dist:
                    best_dist = dist
                    best_idx = i
            if best_idx >= 0:
                used_gt.add(best_idx)
                total_error += best_dist
                matched += 1

        if matched > 0:
            mean_error = total_error / matched
            err_msg = Float32()
            err_msg.data = mean_error
            self.pub_error.publish(err_msg)

            debug = String()
            gt_str = '; '.join(f'{k}=({v[0]:.2f},{v[1]:.2f})' for k, v in self.odom.items())
            det_str = '; '.join(f'({wx:.2f},{wy:.2f}) conf={c:.2f}' for wx, wy, c in detections)
            debug.data = (
                f'detections={len(detections)} matched={matched} '
                f'mean_error={mean_error:.3f}m | GT: {gt_str} | DET: {det_str}'
            )
            self.pub_debug.publish(debug)
            self.get_logger().info(debug.data)


def main(args=None):
    rclpy.init(args=args)
    node = YoloDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
