import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Point
from rclpy.qos import qos_profile_sensor_data
import math


class GetObjectRange(Node):
    def __init__(self):
        super().__init__('get_object_range')

        self.scan_sub = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            qos_profile_sensor_data
        )

        self.object_pub = self.create_publisher(Point, '/object_location', 10)

        # Only search in a front-facing sector. 
        self.front_angle_min = math.radians(-30.0)
        self.front_angle_max = math.radians(30.0)

        # Ignore garbage close returns and anything too far away
        self.min_valid_range = 0.05
        self.max_valid_range = 1.0

        self.get_logger().info('getObjectRange listening to /scan and publishing /object_location')

    def scan_callback(self, msg: LaserScan):
        closest_range = float('inf')
        closest_angle = None

        for i, r in enumerate(msg.ranges):
            angle = msg.angle_min + i * msg.angle_increment

            # Only inspect the forward sector
            if not self.is_angle_in_sector(angle):
                continue

            # Reject invalid measurements
            if math.isinf(r) or math.isnan(r):
                continue

            if r < self.min_valid_range or r > self.max_valid_range:
                continue

            if r < closest_range:
                closest_range = r
                closest_angle = angle

        obstacle_msg = Point()

        # If no obstacle found in the front sector, publish a "far away" marker
        if closest_angle is None:
            obstacle_msg.x = 999.0
            obstacle_msg.y = 0.0
            obstacle_msg.z = 0.0
        else:
            obstacle_msg.x = closest_range * math.cos(closest_angle)
            obstacle_msg.y = closest_range * math.sin(closest_angle)
            obstacle_msg.z = closest_angle

        self.object_pub.publish(obstacle_msg)
        self.get_logger().info(f'obstacle: x={obstacle_msg.x:.3f}, y={obstacle_msg.y:.3f}')

    def is_angle_in_sector(self, angle):
        sector_min = self.front_angle_min
        sector_max = self.front_angle_max
        angle = angle % (2.0 * math.pi)
        sector_min = sector_min % (2.0 * math.pi)
        sector_max = sector_max % (2.0 * math.pi)

        if sector_min <= sector_max:
            return sector_min <= angle <= sector_max

        return angle >= sector_min or angle <= sector_max

def main(args=None):
    rclpy.init(args=args)
    node = GetObjectRange()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()