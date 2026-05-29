"""Coleta 20 amostras do /detection_debug e gera tabela CSV."""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from std_msgs.msg import String, Float32
import csv
import os

SENSOR_QOS = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    history=HistoryPolicy.KEEP_LAST,
    depth=1,
)


class BenchmarkNode(Node):
    def __init__(self):
        super().__init__('benchmark_detector')
        self.samples = []
        self.create_subscription(Float32, '/detection_error', self._cb, SENSOR_QOS)

    def _cb(self, msg):
        self.samples.append(msg.data)
        print(f'  Amostra {len(self.samples):>2}/20  erro={msg.data:.4f}m')


def main():
    rclpy.init()
    node = BenchmarkNode()

    print('\n=== BENCHMARK DO DIGITAL TWIN ===')
    print('Coletando 20 amostras...\n')

    while len(node.samples) < 20:
        rclpy.spin_once(node, timeout_sec=0.5)

    errors = node.samples
    mean_e = sum(errors) / len(errors)
    max_e  = max(errors)
    min_e  = min(errors)

    print(f'\n{"─"*40}')
    print(f'  Amostras : {len(errors)}')
    print(f'  Erro médio: {mean_e:.4f} m  ({mean_e*100:.1f} cm)')
    print(f'  Erro máx  : {max_e:.4f} m')
    print(f'  Erro mín  : {min_e:.4f} m')
    print(f'{"─"*40}\n')

    out = os.path.expanduser('~/Documentos/Projetos/cerise-turtlebot3-nav/docs/benchmark_results.csv')
    with open(out, 'w', newline='') as f:
        w = csv.writer(f)
        w.writerow(['amostra', 'erro_m'])
        for i, e in enumerate(errors, 1):
            w.writerow([i, f'{e:.4f}'])
    print(f'CSV salvo em: {out}')

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
