import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import random, json, time

WAYPOINTS = {
    'A': (0.0,  0.0),
    'B': (1.5,  0.0),
    'C': (0.0, -1.5),
    'D': (1.5, -1.5),
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
