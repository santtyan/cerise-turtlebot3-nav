"""
Move robot1 e robot2 com cmd_vel aleatorio.
Sem Nav2 — apenas publica velocidades para gerar dataset dinamico.

Limites do workspace: |x|, |y| < 1.4m. Ao bater no limite, vira 180.
"""
import math
import random

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry


WORKSPACE_LIMIT = 1.4   # metros
PUB_RATE_HZ     = 5.0
SWITCH_MIN_S    = 3.0
SWITCH_MAX_S    = 5.0
LIN_MIN, LIN_MAX = 0.05, 0.18
ANG_MIN, ANG_MAX = -0.6, 0.6


class RandomDriver(Node):
    def __init__(self, robot_name: str):
        super().__init__(f'random_driver_{robot_name}')
        self.robot = robot_name
        self.pub = self.create_publisher(Twist, f'/{robot_name}/cmd_vel', 10)
        self.create_subscription(
            Odometry, f'/{robot_name}/odom', self._odom_cb, 10
        )
        self.x = 0.0
        self.y = 0.0
        self.next_switch = self.get_clock().now().nanoseconds / 1e9
        self.cur_lin = 0.1
        self.cur_ang = 0.0
        self.create_timer(1.0 / PUB_RATE_HZ, self._tick)
        self.get_logger().info(f'RandomDriver para {robot_name} iniciado')

    def _odom_cb(self, msg: Odometry):
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y

    def _new_command(self):
        self.cur_lin = random.uniform(LIN_MIN, LIN_MAX)
        self.cur_ang = random.uniform(ANG_MIN, ANG_MAX)
        self.next_switch = (
            self.get_clock().now().nanoseconds / 1e9
            + random.uniform(SWITCH_MIN_S, SWITCH_MAX_S)
        )

    def _tick(self):
        t = self.get_clock().now().nanoseconds / 1e9
        cmd = Twist()

        # Se passou do tempo, sortear novo cmd
        if t >= self.next_switch:
            self._new_command()

        # Se passou do limite do workspace, virar (forte angular, sem linear)
        if abs(self.x) > WORKSPACE_LIMIT or abs(self.y) > WORKSPACE_LIMIT:
            cmd.linear.x = -0.05
            cmd.angular.z = 1.0 if self.cur_ang >= 0 else -1.0
            # Forçar troca rápida apos sair do limite
            self.next_switch = t + 1.0
        else:
            cmd.linear.x = self.cur_lin
            cmd.angular.z = self.cur_ang

        self.pub.publish(cmd)


def main():
    rclpy.init()
    nodes = [RandomDriver('robot1'), RandomDriver('robot2')]
    executor = rclpy.executors.MultiThreadedExecutor()
    for n in nodes:
        executor.add_node(n)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        for n in nodes:
            n.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
