import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage

from geometry_msgs.msg import Point
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy, QoSHistoryPolicy

import numpy as np
import cv2


class FindObject(Node):

    def __init__(self):
        super().__init__('find_object')

        # Parameters
        self.declare_parameter('show_image_bool', False)
        self.declare_parameter('window_name', "Raw Image")

        self._display_image = bool(self.get_parameter('show_image_bool').value)
        self._titleOriginal = self.get_parameter('window_name').value

        if self._display_image:
            cv2.namedWindow(self._titleOriginal, cv2.WINDOW_AUTOSIZE)
            cv2.moveWindow(self._titleOriginal, 50, 50)

        # CONFIG
        self.RESIZE_W = 640
        self.RESIZE_H = 480

        # Yellow HSV bounds
        self.LOWER = np.array([20, 100, 100])
        self.UPPER = np.array([35, 255, 255])

        self.MIN_AREA = 1200
        self.KERNEL_SIZE = 5
        self.PRINT_EVERY_N_FRAMES = 5

        self.kernel = np.ones((self.KERNEL_SIZE, self.KERNEL_SIZE), np.uint8)

        # QoS Profile
        image_qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            durability=QoSDurabilityPolicy.VOLATILE,
            depth=1
        )

        self._video_subscriber = self.create_subscription(
            CompressedImage,
            '/image_raw/compressed',
            self._image_callback,
            image_qos_profile
        )

        self.location_publisher = self.create_publisher(Point, '/object_pixel', 10)

    def _image_callback(self, msg: CompressedImage):

        np_arr = np.frombuffer(msg.data, np.uint8)
        self._imgBGR = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)

        if self._imgBGR is None:
            return

        self._imgBGR = cv2.resize(self._imgBGR, (self.RESIZE_W, self.RESIZE_H))
        hsv = cv2.cvtColor(self._imgBGR, cv2.COLOR_BGR2HSV)

        mask = cv2.inRange(hsv, self.LOWER, self.UPPER)

        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, self.kernel, iterations=1)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, self.kernel, iterations=1)

        cnt = self.find_largest_contour(mask)

        found = False
        px = 0
        py = 0

        if cnt is not None:
            area = cv2.contourArea(cnt)

            if area >= self.MIN_AREA:
                found = True
                cxcy = self.contour_centroid(cnt)

                if cxcy is not None:
                    cx, cy = cxcy
                    px = cx - self.RESIZE_W / 2
                    py = cy - self.RESIZE_H / 2

                    # cv2.circle(self._imgBGR, (cx, cy), 6, (0, 0, 255), -1)
        loc_msg = Point()
        loc_msg.x = float(px)
        loc_msg.y = float(py)
        self.location_publisher.publish(loc_msg)
        self.get_logger().info('Publishing: x=%.2f, y=%.2f' %(loc_msg.x, loc_msg.y))

        # if self._display_image:
        #     self.show_image(self._imgBGR)

    def show_image(self, img):
        cv2.imshow(self._titleOriginal, img)
        self._user_input = cv2.waitKey(50)

    def find_largest_contour(self, mask: np.ndarray):
        contours_info = cv2.findContours(
            mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE
        )
        contours = contours_info[0] if len(contours_info) == 2 else contours_info[1]

        if not contours:
            return None

        return max(contours, key=cv2.contourArea)

    def contour_centroid(self, cnt):
        M = cv2.moments(cnt)

        if M["m00"] == 0:
            return None

        cx = int(M["m10"] / M["m00"])
        cy = int(M["m01"] / M["m00"])

        return cx, cy

    def get_user_input(self):
        return self._user_input


def main():
    rclpy.init()
    video_subscriber = FindObject()

    while rclpy.ok():
        rclpy.spin_once(video_subscriber)

        # if video_subscriber._display_image:
        #     if video_subscriber.get_user_input() == ord('q'):
        #         cv2.destroyAllWindows()
        #         break

    rclpy.logging.get_logger("Camera Viewer Node Info...").info("Shutting Down")

    video_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
