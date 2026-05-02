"""
Envia goals de navegação aleatórios para robot1 e robot2 via Nav2 action.
Robôs se movem pelo labirinto, gerando diversidade de posições para o dataset YOLO.
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped
import random
import math


# Goals válidos dentro do labirinto turtlebot3_world (metros)
WAYPOINTS = [
    ( 0.0,  0.0),
    ( 1.0,  0.5),
    ( 1.0, -0.5),
    (-0.5,  1.0),
    (-0.5, -1.0),
    ( 1.5,  0.0),
    ( 0.5,  1.5),
    ( 0.0, -1.5),
]


class RandomGoalSender(Node):
    def __init__(self, robot_name: str):
        super().__init__(f'goal_sender_{robot_name}')
        self.robot = robot_name
        self.client = ActionClient(self, NavigateToPose, f'/{robot_name}/navigate_to_pose')
        self.client.wait_for_server(timeout_sec=30.0)
        self.get_logger().info(f'{robot_name}: Nav2 conectado')
        self._send_next()

    def _send_next(self):
        x, y = random.choice(WAYPOINTS)
        yaw = random.uniform(-math.pi, math.pi)

        goal = NavigateToPose.Goal()
        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.orientation.z = math.sin(yaw / 2)
        pose.pose.orientation.w = math.cos(yaw / 2)
        goal.pose = pose

        self.get_logger().info(f'{self.robot} → goal ({x:.1f}, {y:.1f})')
        future = self.client.send_goal_async(goal)
        future.add_done_callback(self._goal_accepted)

    def _goal_accepted(self, future):
        handle = future.result()
        if not handle.accepted:
            self.get_logger().warn(f'{self.robot}: goal rejeitado, tentando próximo...')
            self._send_next()
            return
        handle.get_result_async().add_done_callback(lambda f: self._send_next())


def main():
    rclpy.init()
    r1 = RandomGoalSender('robot1')
    r2 = RandomGoalSender('robot2')

    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(r1)
    executor.add_node(r2)

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        r1.destroy_node()
        r2.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
