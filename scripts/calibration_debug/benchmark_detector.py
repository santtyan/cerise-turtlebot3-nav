"""
Benchmark do yolo_detector: teleporta robôs para 8 poses e mede erro de detecção.
Gera tabela CSV e imprime resultado no terminal.

Uso: python3 scripts/calibration_debug/benchmark_detector.py
(Gazebo + yolo_detector devem estar rodando)
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from gazebo_msgs.srv import SetEntityState
from gazebo_msgs.msg import EntityState
from std_msgs.msg import Float32, String
from geometry_msgs.msg import Pose
import math
import time
import csv
import os

SENSOR_QOS = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    history=HistoryPolicy.KEEP_LAST,
    depth=1,
)

# 8 poses de teste (robot1_x, robot1_y, robot2_x, robot2_y)
TEST_POSES = [
    ( 0.00,  0.50,  0.00, -0.50),
    ( 0.80,  0.80, -0.80, -0.80),
    (-0.50,  1.20,  0.50, -1.20),
    ( 1.20,  0.00, -1.20,  0.00),
    ( 0.30,  1.50,  0.30, -1.50),
    (-1.00,  0.50,  1.00, -0.50),
    ( 1.50,  1.00, -1.50, -1.00),
    ( 0.00,  1.70,  0.00, -1.70),
]


class BenchmarkNode(Node):
    def __init__(self):
        super().__init__('benchmark_detector')
        self.last_error = None
        self.last_debug = None

        self.create_subscription(Float32, '/detection_error',
                                 lambda m: setattr(self, 'last_error', m.data), SENSOR_QOS)
        self.create_subscription(String, '/detection_debug',
                                 lambda m: setattr(self, 'last_debug', m.data), SENSOR_QOS)

        self.teleport_cli = self.create_client(SetEntityState, '/set_entity_state')
        while not self.teleport_cli.wait_for_service(timeout_sec=3.0):
            self.get_logger().info('Aguardando /set_entity_state...')

    def teleport(self, name, x, y):
        req = SetEntityState.Request()
        req.state = EntityState()
        req.state.name = name
        req.state.pose = Pose()
        req.state.pose.position.x = x
        req.state.pose.position.y = y
        req.state.pose.position.z = 0.01
        req.state.pose.orientation.w = 1.0
        self.teleport_cli.call_async(req)

    def measure(self, r1x, r1y, r2x, r2y, wait=2.0):
        self.teleport('robot1', r1x, r1y)
        self.teleport('robot2', r2x, r2y)
        self.last_error = None
        deadline = time.time() + wait
        while time.time() < deadline:
            rclpy.spin_once(self, timeout_sec=0.1)
        return self.last_error


def main():
    rclpy.init()
    node = BenchmarkNode()

    print('\n=== BENCHMARK DO DIGITAL TWIN ===\n')
    print(f'{"Pose":<6} {"R1 GT":>14} {"R2 GT":>14} {"Erro (m)":>10}')
    print('-' * 50)

    results = []
    errors = []

    for i, (r1x, r1y, r2x, r2y) in enumerate(TEST_POSES):
        error = node.measure(r1x, r1y, r2x, r2y, wait=2.5)
        err_str = f'{error:.3f}' if error is not None else 'N/A'
        print(f'{i+1:<6} ({r1x:+.2f},{r1y:+.2f})  ({r2x:+.2f},{r2y:+.2f})  {err_str:>10}')
        results.append({'pose': i+1, 'r1x': r1x, 'r1y': r1y, 'r2x': r2x, 'r2y': r2y, 'error_m': err_str})
        if error is not None:
            errors.append(error)

    print('-' * 50)
    if errors:
        print(f'Erro médio: {sum(errors)/len(errors):.3f}m | '
              f'Erro máx: {max(errors):.3f}m | '
              f'Detecções: {len(errors)}/{len(TEST_POSES)}')

    # Salva CSV
    out = os.path.join(os.path.dirname(__file__), '..', '..', 'docs', 'benchmark_results.csv')
    out = os.path.realpath(out)
    with open(out, 'w', newline='') as f:
        writer = csv.DictWriter(f, fieldnames=['pose', 'r1x', 'r1y', 'r2x', 'r2y', 'error_m'])
        writer.writeheader()
        writer.writerows(results)
    print(f'\nResultados salvos em: {out}')

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
