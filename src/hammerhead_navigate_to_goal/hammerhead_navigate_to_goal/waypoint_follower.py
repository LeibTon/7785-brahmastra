import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

import math
import time


class WaypointFollower(Node):

    def __init__(self):
        super().__init__('waypoint_follower')

        # Publisher
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        # Subscriber
        self.odom_sub = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            10
        )

        # Robot pose
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0

        # Load waypoints
        self.waypoints = self.load_waypoints("wayPoints.txt")

        self.current_goal = 0

        # Control gains
        self.k_linear = 0.6
        self.k_angular = 1.5

        # Max speeds
        self.max_linear = 0.22
        self.max_angular = 2.0

        # Goal tolerance
        self.goal_tolerance = 0.1

        # Timer loop
        self.timer = self.create_timer(0.1, self.control_loop)

        self.get_logger().info("Waypoint follower started")

    def load_waypoints(self, filename):

        waypoints = []

        with open(filename, 'r') as file:
            for line in file:
                x, y = map(float, line.split())
                waypoints.append((x, y))

        self.get_logger().info(f"Loaded {len(waypoints)} waypoints")

        return waypoints

    def odom_callback(self, msg):

        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y

        q = msg.pose.pose.orientation

        siny = 2 * (q.w*q.z + q.x*q.y)
        cosy = 1 - 2 * (q.y*q.y + q.z*q.z)

        self.theta = math.atan2(siny, cosy)

    def normalize_angle(self, angle):

        while angle > math.pi:
            angle -= 2*math.pi

        while angle < -math.pi:
            angle += 2*math.pi

        return angle

    def control_loop(self):

        if self.current_goal >= len(self.waypoints):

            self.stop_robot()
            return

        goal_x, goal_y = self.waypoints[self.current_goal]

        dx = goal_x - self.x
        dy = goal_y - self.y

        distance = math.sqrt(dx**2 + dy**2)

        goal_theta = math.atan2(dy, dx)

        heading_error = self.normalize_angle(goal_theta - self.theta)

        twist = Twist()

        # Check goal reached
        if distance < self.goal_tolerance:

            self.get_logger().info(f"Reached waypoint {self.current_goal}")

            self.stop_robot()

            time.sleep(10)

            self.current_goal += 1
            return

        # Control law
        linear = self.k_linear * distance
        angular = self.k_angular * heading_error

        # Clamp speeds
        linear = max(min(linear, self.max_linear), -self.max_linear)
        angular = max(min(angular, self.max_angular), -self.max_angular)

        twist.linear.x = linear
        twist.angular.z = angular

        self.cmd_pub.publish(twist)

    def stop_robot(self):

        twist = Twist()
        self.cmd_pub.publish(twist)


def main(args=None):

    rclpy.init(args=args)

    node = WaypointFollower()

    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()