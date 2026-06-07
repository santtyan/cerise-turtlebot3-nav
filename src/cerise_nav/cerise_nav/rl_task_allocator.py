"""Nó ROS2 de alocação de tarefas guiado por política PPO (gêmeo digital).

Substitui a decisão fixa de `task_allocator.py` (nearest_free) por inferência de
uma política PPO treinada offline no AllocationEnv. Toda a infraestrutura
(fila, envio de goal ao Nav2, medição de latência, log CSV) é idêntica ao
task_allocator original — só a ESCOLHA do robô muda.

Estado (observação) montado a partir das detecções do YOLO (`/robot_detections`)
com identidade resolvida por associação às odometrias (`/{r}/odom`). Se um robô
não casar com nenhuma detecção (oclusão/fora do FOV), usa odom como fallback.
Logamos a fração de fallback para reportar no paper.

Parâmetros ROS:
  - model_path (str): caminho do .zip do PPO. Default: models/ppo_allocator_yolo.zip
  - obs_source (str): 'yolo' (default) ou 'odom' — para a ablação no Gazebo.
"""

import csv
import json
import math
import os
import time

import numpy as np
import rclpy
from geometry_msgs.msg import PoseArray, PoseStamped
from nav_msgs.msg import Odometry
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from rclpy.node import Node
from std_msgs.msg import String

from cerise_nav.rl import obs_encoding

LOG_FILE = os.path.expanduser('~/cerise_log.csv')
ROBOTS = ['robot1', 'robot2', 'robot3']  # ampliável; ajustar ao cenário ativo
# Distância máxima (m) para associar uma detecção a um robô via odom.
ASSOC_MAX_DIST = 0.5


class RLTaskAllocator(Node):

    def __init__(self):
        super().__init__('rl_task_allocator')

        self.declare_parameter('model_path', _default_model_path())
        self.declare_parameter('obs_source', 'yolo')
        self.declare_parameter('robots', ROBOTS)
        model_path = self.get_parameter('model_path').value
        self.obs_source = self.get_parameter('obs_source').value
        self.robots = list(self.get_parameter('robots').value)
        self.num_robots = len(self.robots)

        # Carrega a política PPO (import tardio para não exigir SB3 se o nó
        # nunca for instanciado durante testes de outros nós).
        from stable_baselines3 import PPO
        if not os.path.exists(model_path):
            raise FileNotFoundError(f'modelo PPO não encontrado: {model_path}')
        self.model = PPO.load(model_path)
        self.get_logger().info(f'política carregada: {model_path} '
                               f'(obs_source={self.obs_source})')

        self.odom = {r: (0.0, 0.0) for r in self.robots}
        self.detections = []          # [(x, y), ...] do YOLO
        self.busy = {r: False for r in self.robots}
        # busy_start[r] = tempo de início da tarefa atual (para busy_remaining).
        self.busy_start = {r: None for r in self.robots}
        self.queue = []

        # Métrica de qualidade para o paper: fração de obs que usaram fallback.
        self._fallback_count = 0
        self._assoc_count = 0

        self.create_subscription(String, '/demands', self.on_demand, 10)
        self.create_subscription(
            PoseArray, '/robot_detections', self.on_detections, 10)
        for r in self.robots:
            self.create_subscription(
                Odometry, f'/{r}/odom',
                lambda msg, robot=r: self.on_odom(msg, robot), 10)

        self.nav_clients = {
            r: ActionClient(self, NavigateToPose, f'/{r}/navigate_to_pose')
            for r in self.robots}

        self.init_csv()
        self.get_logger().info('RLTaskAllocator iniciado')

    # ------------------------------------------------------------ callbacks
    def on_odom(self, msg, robot):
        self.odom[robot] = (msg.pose.pose.position.x, msg.pose.pose.position.y)

    def on_detections(self, msg):
        self.detections = [(p.position.x, p.position.y) for p in msg.poses]

    def on_demand(self, msg):
        demand = json.loads(msg.data)
        self.queue.append(demand)
        self.get_logger().info(f"Demanda {demand['id']} | fila: {len(self.queue)}")
        self.try_dispatch()

    # ------------------------------------------------------------- dispatch
    def try_dispatch(self):
        if not self.queue:
            return
        demand = self.queue[0]
        obs = self.build_observation(demand['origin_xy'])

        # Inferência PPO síncrona e instantânea — não bloqueia o executor.
        action, _ = self.model.predict(obs, deterministic=True)
        robot = self.robots[int(action)]

        if self.busy[robot]:
            # A política escolheu um robô ocupado. Mantém a demanda na fila e
            # tenta de novo quando algum robô liberar (em on_result).
            self.get_logger().info(
                f'{robot} ocupado (escolha do PPO); aguardando liberar.')
            return

        self.queue.pop(0)
        self.busy[robot] = True
        self.busy_start[robot] = time.time()
        self.get_logger().info(
            f"{robot} -> {demand['dest']} (demanda {demand['id']}) [PPO]")
        self.send_goal(robot, demand)

    def build_observation(self, demand_origin):
        """Monta a observação no MESMO layout do AllocationEnv (obs_encoding).

        Posições por robô vêm do YOLO (com identidade resolvida via odom) ou da
        odom pura, conforme obs_source. busy_remaining é estimado pelo tempo
        decorrido desde o início da tarefa atual.
        """
        est_pos = self.estimate_positions()
        positions = [est_pos[r] for r in self.robots]
        busy_remaining = [self._busy_remaining(r) for r in self.robots]
        return obs_encoding.encode_obs(positions, busy_remaining, demand_origin)

    def estimate_positions(self):
        """Posição estimada por robô conforme obs_source.

        'odom': usa odom direto.
        'yolo': associa detecções YOLO aos robôs por vizinho-mais-próximo da
                odom; fallback para odom quando não há detecção próxima.
        """
        if self.obs_source == 'odom' or not self.detections:
            if self.obs_source == 'yolo':
                # Sem detecções neste instante: tudo fallback.
                self._fallback_count += self.num_robots
                self._assoc_count += self.num_robots
            return dict(self.odom)

        # Associação greedy detecção -> robô (mesmo padrão de yolo_detector).
        est = {}
        used = set()
        for r in self.robots:
            ox, oy = self.odom[r]
            best_d, best_j = float('inf'), -1
            for j, (dx, dy) in enumerate(self.detections):
                if j in used:
                    continue
                d = math.hypot(dx - ox, dy - oy)
                if d < best_d:
                    best_d, best_j = d, j
            self._assoc_count += 1
            if best_j >= 0 and best_d <= ASSOC_MAX_DIST:
                used.add(best_j)
                est[r] = self.detections[best_j]
            else:
                est[r] = self.odom[r]          # fallback
                self._fallback_count += 1
        return est

    def _busy_remaining(self, robot):
        """Estimativa grosseira do tempo restante de ocupação (s)."""
        if not self.busy[robot] or self.busy_start[robot] is None:
            return 0.0
        # Sem previsão de fim; usa tempo já decorrido como proxy negativo:
        # retornamos um valor positivo decrescente baseado num teto típico.
        elapsed = time.time() - self.busy_start[robot]
        return max(0.0, obs_encoding.BUSY_REF - elapsed)

    # ----------------------------------------------------------- Nav2 chain
    def send_goal(self, robot, demand):
        client = self.nav_clients[robot]
        if not client.server_is_ready():
            self.get_logger().warning(f'{robot}: Nav2 nao pronto, retry em 2s')
            self.busy[robot] = False
            self.queue.insert(0, demand)
            self.create_timer(2.0, lambda: self.try_dispatch())
            return
        goal = NavigateToPose.Goal()
        goal.pose = PoseStamped()
        goal.pose.header.frame_id = 'map'
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x = float(demand['dest_xy'][0])
        goal.pose.pose.position.y = float(demand['dest_xy'][1])
        goal.pose.pose.orientation.w = 1.0
        t0 = time.time()
        future = client.send_goal_async(goal)
        future.add_done_callback(
            lambda f, r=robot, d=demand, t=t0: self.on_accepted(f, r, d, t))

    def on_accepted(self, future, robot, demand, t0):
        handle = future.result()
        if not handle.accepted:
            self.get_logger().error(f'{robot}: goal rejeitado')
            self.busy[robot] = False
            return
        handle.get_result_async().add_done_callback(
            lambda f, r=robot, d=demand, t=t0: self.on_result(f, r, d, t))

    def on_result(self, future, robot, demand, t0):
        latency = time.time() - t0
        self.get_logger().info(
            f"[LATENCIA] {robot} demanda {demand['id']}: {latency:.2f}s")
        self.log_csv(demand, robot, latency)
        self.busy[robot] = False
        self.busy_start[robot] = None
        self.try_dispatch()

    # ------------------------------------------------------------------ CSV
    def init_csv(self):
        if not os.path.exists(LOG_FILE):
            with open(LOG_FILE, 'w', newline='') as f:
                csv.writer(f).writerow(
                    ['timestamp', 'demand_id', 'origin', 'dest', 'robot',
                     'latency_s', 'policy'])

    def log_csv(self, demand, robot, latency):
        fb = (self._fallback_count / self._assoc_count
              if self._assoc_count else 0.0)
        with open(LOG_FILE, 'a', newline='') as f:
            csv.writer(f).writerow([
                time.strftime('%Y-%m-%d %H:%M:%S'),
                demand['id'], demand['origin'], demand['dest'],
                robot, round(latency, 2), f'ppo_{self.obs_source}'])
        self.get_logger().info(f'[FALLBACK] fração acumulada: {fb:.2%}')


def _default_model_path():
    here = os.path.dirname(os.path.abspath(__file__))
    repo = os.path.abspath(os.path.join(here, '..', '..', '..'))
    return os.path.join(repo, 'models', 'ppo_allocator_yolo.zip')


def main(args=None):
    rclpy.init(args=args)
    node = RLTaskAllocator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
