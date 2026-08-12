"""EKF de fusão câmera(YOLO)+odometria por robô — substitui o fallback discreto
obs_source em allocation_env.py/rl_task_allocator.py:146-178.

Estado por robô: [px, py, theta]. Predição via odometria (Q), correção via
detecção YOLO com R variável por confiança (box.conf[0]). Design validado
sinteticamente via NEES/NIS em scripts/validate_ekf_synthetic.py (12/08/2026).

Uso: ros2 run cerise_nav ekf_fusion_node
"""

import math

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseArray, PoseWithCovarianceStamped

from cerise_nav.association import mahalanobis_gate


SENSOR_QOS = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    history=HistoryPolicy.KEEP_LAST,
    depth=1,
)

# Q/R calibrados via validate_ekf_synthetic.py (Monte Carlo NEES/NIS, 30 seeds)
Q_DIAG = (1e-6, 1e-6, 5e-7)
R_MIN, R_MAX = 0.02, 0.20  # metros, escala por confiança do YOLO


def r_from_confidence(conf: float) -> np.ndarray:
    conf = np.clip(conf, 0.05, 1.0)
    sigma = R_MIN + (1.0 - conf) * (R_MAX - R_MIN)
    return np.diag([sigma ** 2, sigma ** 2])


class RobotEKF:
    """EKF isolado por robô: estado [px, py, theta]."""

    def __init__(self, initial_state: np.ndarray):
        self.state = initial_state.copy()
        self.cov = np.diag([0.01, 0.01, 0.01])
        self._last_odom = None  # (x, y, theta, stamp) para derivar v, w

    def predict_from_odom(self, x, y, theta, dt):
        if dt <= 0:
            return
        F = np.eye(3)  # aproximação linear simples para odometria absoluta
        Q = np.diag(Q_DIAG)
        self.state = np.array([x, y, theta])
        self.cov = F @ self.cov @ F.T + Q

    def correct_with_detection(self, det_xy, conf):
        H = np.array([[1.0, 0.0, 0.0], [0.0, 1.0, 0.0]])
        R = r_from_confidence(conf)
        innovation = np.array(det_xy) - H @ self.state
        S = H @ self.cov @ H.T + R
        K = self.cov @ H.T @ np.linalg.inv(S)
        self.state = self.state + K @ innovation
        self.cov = (np.eye(3) - K @ H) @ self.cov


class EkfFusionNode(Node):
    def __init__(self):
        super().__init__('ekf_fusion_node')

        self.declare_parameter('robots', ['robot1', 'robot2', 'robot3'])
        self.robots = list(self.get_parameter('robots').value)

        self.filters = {}
        self._odom_raw = {}
        self._last_odom_time = {}
        self._detections = []

        for r in self.robots:
            self.create_subscription(
                Odometry, f'/{r}/odom',
                lambda m, rid=r: self._odom_cb(rid, m), SENSOR_QOS)

        self.create_subscription(PoseArray, '/robot_detections', self._detections_cb, 10)

        self.pub_fused = {
            r: self.create_publisher(PoseWithCovarianceStamped, f'/{r}/ekf_pose', 10)
            for r in self.robots
        }

        self.get_logger().info(f'EkfFusionNode pronto para {self.robots}')

    def _odom_cb(self, robot_id, msg: Odometry):
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        theta = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                            1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        now = self.get_clock().now().nanoseconds / 1e9

        if robot_id not in self.filters:
            self.filters[robot_id] = RobotEKF(np.array([p.x, p.y, theta]))
            self._last_odom_time[robot_id] = now
            return

        dt = now - self._last_odom_time[robot_id]
        self._last_odom_time[robot_id] = now
        self.filters[robot_id].predict_from_odom(p.x, p.y, theta, dt)
        self._odom_raw[robot_id] = (p.x, p.y)

        self._publish(robot_id)

    def _detections_cb(self, msg: PoseArray):
        # position.z carrega box.conf[0] do YOLO (ver yolo_detector.py:_publish_detections)
        self._detections = [(pose.position.x, pose.position.y) for pose in msg.poses]
        det_conf = {(pose.position.x, pose.position.y): pose.position.z for pose in msg.poses}
        if not self._odom_raw or not self.filters:
            return

        cov_by_robot = {r: f.cov for r, f in self.filters.items() if r in self._odom_raw}
        assignments, _unmatched = mahalanobis_gate(
            self._odom_raw, cov_by_robot, self._detections)

        for robot_id, det_xy in assignments.items():
            conf = det_conf.get(det_xy, 0.5)  # fallback conservador se chave não bater
            self.filters[robot_id].correct_with_detection(det_xy, conf=conf)
            self._publish(robot_id)

    def _publish(self, robot_id):
        f = self.filters[robot_id]
        msg = PoseWithCovarianceStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'map'
        msg.pose.pose.position.x = float(f.state[0])
        msg.pose.pose.position.y = float(f.state[1])
        msg.pose.pose.orientation.z = math.sin(f.state[2] / 2.0)
        msg.pose.pose.orientation.w = math.cos(f.state[2] / 2.0)
        cov_flat = [0.0] * 36
        cov_flat[0] = float(f.cov[0, 0])
        cov_flat[7] = float(f.cov[1, 1])
        cov_flat[35] = float(f.cov[2, 2])
        msg.pose.covariance = cov_flat
        self.pub_fused[robot_id].publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = EkfFusionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
