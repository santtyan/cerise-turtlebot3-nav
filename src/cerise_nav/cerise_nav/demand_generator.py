import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import random, json, time

# Waypoints nos espaços livres entre os cilindros do turtlebot3_world
# (grade de obstáculos em {-1.1,0,1.1}^2). Todos dentro do FOV da câmera
# overhead (±1.73m H, ±1.3m V com FOV=60°, h=3m). Max coord = ±0.55m → folga.
WAYPOINTS = {
    'A': (-0.55,  0.55),   # NW
    'B': ( 0.55,  0.55),   # NE
    'C': (-0.55, -0.55),   # SW
    'D': ( 0.55, -0.55),   # SE
}

class DemandGenerator(Node):
    def __init__(self):
        super().__init__('demand_generator')
        self.pub = self.create_publisher(String, '/demands', 10)
        self.timer = self.create_timer(30.0, self.publish_demand)
        self.demand_id = 0
        self.get_logger().info('DemandGenerator iniciado')

    def publish_demand(self):
        points = random.sample(list(WAYPOINTS.keys()), 2)
        msg = String()
        msg.data = json.dumps({
            'id': self.demand_id,
            'origin': points[0],
            'origin_xy': WAYPOINTS[points[0]],
            'dest': points[1],
            'dest_xy': WAYPOINTS[points[1]],
            'timestamp': time.time(),
        })
        self.pub.publish(msg)
        self.get_logger().info(f'Demanda {self.demand_id}: {points[0]} -> {points[1]}')
        self.demand_id += 1

def main():
    rclpy.init()
    rclpy.spin(DemandGenerator())
    rclpy.shutdown()
