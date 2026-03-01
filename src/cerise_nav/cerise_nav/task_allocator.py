import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from nav_msgs.msg import Odometry
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from geometry_msgs.msg import PoseStamped
import json, math, time, csv, os
LOG_FILE = os.path.expanduser('~/cerise_log.csv')

ROBOTS = ['robot1', 'robot2']

class TaskAllocator(Node):
    def __init__(self):
        super().__init__('task_allocator')
        self.positions = {r: (0.0, 0.0) for r in ROBOTS}
        self.busy = {r: False for r in ROBOTS}
        self.queue = []
        self.create_subscription(String, '/demands', self.on_demand, 10)
        for r in ROBOTS:
            self.create_subscription(
                Odometry, f'/{r}/odom',
                lambda msg, robot=r: self.on_odom(msg, robot), 10)
        self.nav_clients = {
            r: ActionClient(self, NavigateToPose, f'/{r}/navigate_to_pose')
            for r in ROBOTS}
        self.init_csv()
        self.get_logger().info('TaskAllocator iniciado')

    def on_odom(self, msg, robot):
        self.positions[robot] = (
            msg.pose.pose.position.x,
            msg.pose.pose.position.y)

    def on_demand(self, msg):
        demand = json.loads(msg.data)
        self.queue.append(demand)
        did = demand['id']
        self.get_logger().info(f'Demanda {did} | fila: {len(self.queue)}')
        self.try_dispatch()

    def try_dispatch(self):
        if not self.queue:
            return
        robot = self.nearest_free(self.queue[0]['origin_xy'])
        if robot is None:
            self.get_logger().info('Todos ocupados, aguardando...')
            return
        demand = self.queue.pop(0)
        self.busy[robot] = True
        dest = demand['dest']
        did = demand['id']
        self.get_logger().info(f'{robot} -> {dest} (demanda {did})')
        self.send_goal(robot, demand)

    def nearest_free(self, origin_xy):
        free = [r for r in ROBOTS if not self.busy[r]]
        if not free:
            return None
        ox, oy = origin_xy
        return min(free, key=lambda r: math.dist(self.positions[r], (ox, oy)))

    def send_goal(self, robot, demand):
        client = self.nav_clients[robot]
        if not client.wait_for_server(timeout_sec=2.0):
            self.get_logger().error(f'{robot}: Nav2 indisponivel')
            self.busy[robot] = False
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
        did = demand['id']
        self.get_logger().info(f'[LATENCIA] {robot} demanda {did}: {latency:.2f}s')
        self.log_csv(demand, robot, latency)
        self.busy[robot] = False
        self.try_dispatch()

    def init_csv(self):
        if not os.path.exists(LOG_FILE):
            with open(LOG_FILE, 'w', newline='') as f:
                csv.writer(f).writerow(['timestamp','demand_id','origin','dest','robot','latency_s'])

    def log_csv(self, demand, robot, latency):
        with open(LOG_FILE, 'a', newline='') as f:
            csv.writer(f).writerow([
                time.strftime('%Y-%m-%d %H:%M:%S'),
                demand['id'], demand['origin'], demand['dest'],
                robot, round(latency, 2)
            ])

def main():
    rclpy.init()
    rclpy.spin(TaskAllocator())
    rclpy.shutdown()
