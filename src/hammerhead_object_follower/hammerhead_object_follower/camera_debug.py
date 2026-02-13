import rclpy
from rclpy.node import Node

from sensor_msgs.msg import CompressedImage
import numpy as np
import cv2


class CameraDebug(Node):
    def __init__(self):
        super().__init__('camera_debug')

        # CHANGE THIS to match the topic find_object publishes
        self.image_topic = '/debug_img/compressed'

        self.sub = self.create_subscription(
            CompressedImage,
            self.image_topic,
            self.image_callback,
            10
        )

        self.get_logger().info(f'CameraDebug subscribed to {self.image_topic}')

    def image_callback(self, msg: CompressedImage):
        # msg.data is a bytes array containing a JPEG/PNG (usually JPEG)
        np_arr = np.frombuffer(msg.data, dtype=np.uint8)
        frame = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)

        if frame is None:
            self.get_logger().warn('Failed to decode compressed image frame.')
            return

        cv2.imshow('TurtleBot3 Camera Debug', frame)
        cv2.waitKey(1)  # required for imshow to update

    def destroy_node(self):
        # Cleanly close the OpenCV window when node shuts down
        cv2.destroyAllWindows()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = CameraDebug()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()