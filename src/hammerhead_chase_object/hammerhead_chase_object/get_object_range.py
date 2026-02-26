import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point, Twist
from sensor_msgs.msg import LaserScan
from rclpy.qos import qos_profile_sensor_data
import math

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

        self.timer = self.create_timer(0.1, self.publish_state) # Publish state at 10 Hz

        self.WIDTH_IMAGE = 640
        self.FOV = 62.2 * 3.14159/180 # horizontal FOV of the camera in radians

        self.angle_to_object = 0.0
        self.valid_ang = 0.0
        self.distance_to_object = 0.0
        self.valid_lidar = 0.0
        self.get_logger().info('GetObjectRange listening to /scan and publishing /position and angle of the object.')
    
    def lidar_callback(self, msg: LaserScan):
        if msg.angle_increment == 0.0:
            return  # Prevent division by zero if message is malformed
            
        angle_min = msg.angle_min
        angle_increment = msg.angle_increment
        
        # Calculate the angle difference relative to angle_min
        # Using modulo 2*pi handles wrap-around, ensuring the difference is always positive
        angle_diff = (self.angle_to_object - angle_min) % (2 * math.pi)
        
        # Calculate index and wrap it around the array length just in case
        index = int(round(angle_diff / angle_increment)) % len(msg.ranges)
        
        distance_to_object = msg.ranges[index]
        
        # Check if the LiDAR reading is valid (not inf, nan, and within physical sensor bounds)
        if math.isfinite(distance_to_object) and msg.range_min <= distance_to_object <= msg.range_max:
            self.distance_to_object = distance_to_object
            self.valid_lidar = 1.0
        else:
            self.distance_to_object = 0.0 
            self.valid_lidar = 0.0
            
    def listener_callback(self, msg: Point):
        # find_object already publishes: error_x = object_x - image_center
        error_x = msg.x
        self.valid_ang = msg.z # valid flag from detect_object
        error_rad = -(error_x / self.WIDTH_IMAGE) * self.FOV # Convert pixel error to radians (negated to match LIDAR convention)
        self.pos_x = error_x
        self.angle_to_object = error_rad
        # Normalize angle to [0, 2π] range to match LIDAR convention
        while self.angle_to_object > 3.14159:
            self.angle_to_object -= 2*3.14159
        while self.angle_to_object < -3.14159:
            self.angle_to_object += 2*3.14159
    
    def publish_state(self):
        msg = Point()
        msg.x = self.angle_to_object
        msg.y = self.distance_to_object
        msg.z = 1.0 if (self.valid_ang > 0.5 and self.valid_lidar > 0.5) else 0.0 # valid flag
        self.state_pub.publish(msg)

        self.get_logger().info(f'Published state: angle={self.angle_to_object:.3f} rad, distance={self.distance_to_object:.3f} m, valid={msg.z > 0}')


def main(args=None):
    rclpy.init(args=args)
    node = GetObjectRange()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
