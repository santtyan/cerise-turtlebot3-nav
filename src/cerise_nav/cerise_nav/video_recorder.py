import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import os
import datetime


class VideoRecorder(Node):
    def __init__(self):
        super().__init__('video_recorder')
        self.declare_parameter('output_path', '')
        self.declare_parameter('fps', 10.0)

        out_path = self.get_parameter('output_path').value
        if not out_path:
            ts = datetime.datetime.now().strftime('%Y%m%d_%H%M%S')
            out_path = os.path.expanduser(f'~/cerise_{ts}.mp4')

        fps = self.get_parameter('fps').value
        self.writer = cv2.VideoWriter(
            out_path,
            cv2.VideoWriter_fourcc(*'mp4v'),
            fps,
            (640, 480),
        )
        self.bridge = CvBridge()
        self.create_subscription(
            Image, '/camera/image_raw', self.on_image, qos_profile_sensor_data
        )
        self.get_logger().info(f'Gravando em {out_path} ({fps} fps)')

    def on_image(self, msg):
        frame = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        self.writer.write(frame)

    def destroy_node(self):
        self.writer.release()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = VideoRecorder()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
