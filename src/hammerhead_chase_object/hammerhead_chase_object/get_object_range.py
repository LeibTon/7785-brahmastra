import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point, Twist
from sensor_msgs.msg import LaserScan
from rclpy.qos import qos_profile_sensor_data


class GetObjectRange(Node):
    def __init__(self):
        super().__init__('get_object_range')

        self.state_pub = self.create_publisher(Point, '/robot_state', 10)
        self.object_sub = self.create_subscription(
            Point, '/object_pixel', self.listener_callback, 10
        )
        self.lidar_sub = self.create_subscription(
            LaserScan, '/scan', self.lidar_callback, qos_profile_sensor_data
        )

        # self.timer = self.create_timer(0.1, self.publish_state) # Publish state at 10 Hz

        self.WIDTH_IMAGE = 640
        self.FOV = 62.2 * 3.14159/180 # horizontal FOV of the camera in radians

        self.angle_to_object = 0.0
        self.get_logger().info('GetObjectRange listening to /scan and publishing /position and angle of the object.')
    
    def lidar_callback(self, msg: LaserScan):
        angle_min = msg.angle_min
        angle_max = msg.angle_max
        angle_increment = msg.angle_increment
        ranges = msg.ranges
        index = round((self.angle_to_object - angle_min) / angle_increment)
        if 0 <= index < len(ranges):
            distance_to_object = ranges[index]
            self.get_logger().info(f'Angle to object: {self.angle_to_object:.2f} rad, Distance to object: {distance_to_object:.2f} m')
        else:
            self.get_logger().warn('Calculated index for LiDAR range is out of bounds.')


    def listener_callback(self, msg: Point):
        # find_object already publishes: error_x = object_x - image_center
        error_x = msg.x
        error_rad = -(error_x / self.WIDTH_IMAGE) * self.FOV # Convert pixel error to radians (negated to match LIDAR convention)

        self.angle_to_object = error_rad
        # Normalize angle to [0, 2π] range to match LIDAR convention
        while self.angle_to_object > 2 * 3.14159:
            self.angle_to_object -= 2*3.14159
        while self.angle_to_object < 0:
            self.angle_to_object += 2*3.14159
    
    def publish_state(self):
        print(self.angle_to_object)


def main(args=None):
    rclpy.init(args=args)
    node = GetObjectRange()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
