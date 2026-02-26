import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point, Twist


class ChaseObject(Node):
    def __init__(self):
        super().__init__('chase_object')

        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.object_sub = self.create_subscription(
            Point, '/object_pixel', self.listener_callback, 10
        )

        self.get_logger().info('ChaseObject listening to /object_pixel and publishing /cmd_vel.')

    def listener_callback(self, msg: Point):
        # find_object already publishes: error_x = object_x - image_center
        error_x = msg.x

        self.cmd_vel_pub.publish(twist)

        self.get_logger().info(
            f'error_x: {error_x:.1f} px -> angular.z: {twist.angular.z:.3f} rad/s'
        )


def main(args=None):
    rclpy.init(args=args)
    node = ChaseObject()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
