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

# Q recalibrado empiricamente contra os bags reais (passo 4, 12/08/2026) — o
# valor original de validate_ekf_synthetic.py (1e-6) era calibrado para o
# modelo de ruído SINTÉTICO da etapa 1.5 (odometria quase perfeita), não para
# drift real de odometria (POS_DRIFT_ODOM=0.03 em allocation_env.py). Com Q
# pequeno, o filtro nunca corrigia o drift real via YOLO (ganho ~0% ou
# negativo). Varredura em bags reais mostrou ganho crescente até saturar
# ~80% em Q=0.5-1.0. Usar 0.5 como valor default de produção.
Q_DIAG = (0.5, 0.5, 0.25)
R_MIN, R_MAX = 0.02, 0.20  # metros, escala por confiança do YOLO

# Teto de covariância: sem correção por muitos passos (YOLO perdendo detecção
# por oclusão/movimento — reproduzido no cenário 2 dos bags), cov cresce sem
# limite com Q grande, quebrando o gating de Mahalanobis (aceita qualquer
# detecção como "próxima o suficiente", causando "roubo" de detecção entre
# robôs — efeito documentado na referência #4, Altendorfer & Wirkert 2015).
# Achado desta sessão: cov chegou a 85 (deveria ~0.03-0.05) num evento real.
COV_CAP = 0.05


def r_from_confidence(conf: float) -> np.ndarray:
    conf = np.clip(conf, 0.05, 1.0)
    sigma = R_MIN + (1.0 - conf) * (R_MAX - R_MIN)
    return np.diag([sigma ** 2, sigma ** 2])


class RobotEKF:
    """EKF isolado por robô: estado [px, py, theta].

    Predição integra o DELTA de odometria (movimento relativo) sobre o
    estado do próprio filtro, nunca substitui o estado pelo valor absoluto
    da odometria — caso contrário a correção do passo anterior (via YOLO)
    é descartada a cada ciclo de predição, anulando o ganho da fusão (bug
    encontrado e corrigido na sessão de 12/08/2026, ver plano LAFusion,
    passo 4: reduzia o ganho do EKF a ~0.2-0.4% mesmo com drift agressivo).
    """

    def __init__(self, initial_state: np.ndarray):
        self.state = initial_state.copy()
        self.cov = np.diag([0.01, 0.01, 0.01])
        self._last_odom = None  # (x, y, theta) da última leitura bruta de odometria
        self._last_t = None

    def predict_from_odom(self, x, y, theta, t):
        if self._last_odom is None:
            self._last_odom = (x, y, theta)
            self._last_t = t
            return

        lx, ly, ltheta = self._last_odom
        dx, dy, dtheta = x - lx, y - ly, theta - ltheta
        dtheta = math.atan2(math.sin(dtheta), math.cos(dtheta))  # wrap
        dt = max(t - self._last_t, 1e-6)
        self._last_odom = (x, y, theta)
        self._last_t = t

        # Q calibrado para dt=0.1s — escalar pela razão dt_real/0.1 corrige a
        # taxa real de odometria (~29Hz medida nos bags).
        Q = np.diag(Q_DIAG) * (dt / 0.1)

        F = np.eye(3)
        self.state = self.state + np.array([dx, dy, dtheta])
        self.state[2] = math.atan2(math.sin(self.state[2]), math.cos(self.state[2]))
        self.cov = F @ self.cov @ F.T + Q
        np.clip(self.cov, None, COV_CAP, out=self.cov)

    def correct_with_detection(self, det_xy, conf):
        H = np.array([[1.0, 0.0, 0.0], [0.0, 1.0, 0.0]])
        R = r_from_confidence(conf)
        innovation = np.array(det_xy) - H @ self.state
        S = H @ self.cov @ H.T + R
        K = self.cov @ H.T @ np.linalg.inv(S)
        self.state = self.state + K @ innovation
        # Forma de Joseph: preserva simetria e positividade semi-definida de
        # P mesmo sob erro numérico, ao contrário da forma simplificada
        # (I-KH)P, que só é exata para K ótimo sem arredondamento (Bar-Shalom
        # et al., Estimation with Applications to Tracking and Navigation).
        I_KH = np.eye(3) - K @ H
        self.cov = I_KH @ self.cov @ I_KH.T + K @ R @ K.T


class EkfFusionNode(Node):
    def __init__(self):
        super().__init__('ekf_fusion_node')

        self.declare_parameter('robots', ['robot1', 'robot2', 'robot3'])
        self.robots = list(self.get_parameter('robots').value)

        self.filters = {}
        self._odom_raw = {}
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

        if robot_id not in self.filters:
            self.filters[robot_id] = RobotEKF(np.array([p.x, p.y, theta]))
            self._odom_raw[robot_id] = (p.x, p.y)
            return

        now = self.get_clock().now().nanoseconds / 1e9
        self.filters[robot_id].predict_from_odom(p.x, p.y, theta, now)
        self._odom_raw[robot_id] = (p.x, p.y)

        self._publish(robot_id)

    def _detections_cb(self, msg: PoseArray):
        # position.z carrega box.conf[0] do YOLO (ver yolo_detector.py:_publish_detections)
        self._detections = [(pose.position.x, pose.position.y) for pose in msg.poses]
        det_conf = {(pose.position.x, pose.position.y): pose.position.z for pose in msg.poses}
        if not self._odom_raw or not self.filters:
            return

        cov_by_robot = {r: f.cov for r, f in self.filters.items() if r in self._odom_raw}
        r_by_detection = [r_from_confidence(det_conf.get(d, 0.5)) for d in self._detections]
        assignments, _unmatched = mahalanobis_gate(
            self._odom_raw, cov_by_robot, self._detections, r_by_detection=r_by_detection)

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
