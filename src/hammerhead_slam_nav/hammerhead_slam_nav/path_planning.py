import math
import rclpy
from rclpy.node import Node

from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from std_msgs.msg import Header


class WaypointNavigator(Node):
    def __init__(self):
        super().__init__('waypoint_navigator')

        # Publisher to Nav2 goal topic
        self.goal_pub = self.create_publisher(PoseStamped, '/goal_pose', 10)

        # Subscriber to AMCL pose estimate
        self.amcl_sub = self.create_subscription(
            PoseWithCovarianceStamped,
            '/amcl_pose',
            self.amcl_callback,
            10
        )

        # Timer for checking progress
        self.timer = self.create_timer(0.2, self.control_loop)

        # Current robot pose
        self.current_x = None
        self.current_y = None
        self.current_yaw = None

        # Waypoints: (x, y, yaw in radians)
        self.waypoints = [
            (1.0, 0.0, 0.0),
            (2.0, 0.5, math.pi / 2),
            (1.5, -0.5, math.pi)
        ]

        self.current_waypoint_idx = 0
        self.goal_active = False

        # Tolerances
        self.position_tolerance = 0.20   # meters
        self.yaw_tolerance = 0.30        # radians

        self.get_logger().info('Waypoint navigator started.')

    def amcl_callback(self, msg: PoseWithCovarianceStamped):
        self.current_x = msg.pose.pose.position.x
        self.current_y = msg.pose.pose.position.y

        q = msg.pose.pose.orientation
        self.current_yaw = self.quaternion_to_yaw(q.x, q.y, q.z, q.w)

    def control_loop(self):
        # Wait until AMCL pose is available
        if self.current_x is None or self.current_y is None or self.current_yaw is None:
            return

        # If all waypoints are done
        if self.current_waypoint_idx >= len(self.waypoints):
            if self.goal_active:
                self.get_logger().info('All waypoints completed.')
                self.goal_active = False
            return

        # If no goal is active, publish the next one
        if not self.goal_active:
            self.publish_goal(*self.waypoints[self.current_waypoint_idx])
            self.goal_active = True
            return

        # Check if current waypoint is reached
        goal_x, goal_y, goal_yaw = self.waypoints[self.current_waypoint_idx]

        pos_error = math.sqrt((goal_x - self.current_x) ** 2 + (goal_y - self.current_y) ** 2)
        yaw_error = self.angle_diff(goal_yaw, self.current_yaw)

        if pos_error < self.position_tolerance and abs(yaw_error) < self.yaw_tolerance:
            self.get_logger().info(
                f'Reached waypoint {self.current_waypoint_idx + 1}: '
                f'({goal_x:.2f}, {goal_y:.2f}, {goal_yaw:.2f})'
            )
            self.current_waypoint_idx += 1
            self.goal_active = False

    def publish_goal(self, x: float, y: float, yaw: float):
        goal_msg = PoseStamped()
        goal_msg.header = Header()
        goal_msg.header.stamp = self.get_clock().now().to_msg()
        goal_msg.header.frame_id = 'map'

        goal_msg.pose.position.x = x
        goal_msg.pose.position.y = y
        goal_msg.pose.position.z = 0.0

        qx, qy, qz, qw = self.yaw_to_quaternion(yaw)
        goal_msg.pose.orientation.x = qx
        goal_msg.pose.orientation.y = qy
        goal_msg.pose.orientation.z = qz
        goal_msg.pose.orientation.w = qw

        self.goal_pub.publish(goal_msg)
        self.get_logger().info(
            f'Published waypoint {self.current_waypoint_idx + 1}: '
            f'({x:.2f}, {y:.2f}, {yaw:.2f})'
        )

    @staticmethod
    def yaw_to_quaternion(yaw: float):
        qx = 0.0
        qy = 0.0
        qz = math.sin(yaw / 2.0)
        qw = math.cos(yaw / 2.0)
        return qx, qy, qz, qw

    @staticmethod
    def quaternion_to_yaw(x: float, y: float, z: float, w: float):
        siny_cosp = 2.0 * (w * z + x * y)
        cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
        return math.atan2(siny_cosp, cosy_cosp)

    @staticmethod
    def angle_diff(a: float, b: float):
        diff = a - b
        while diff > math.pi:
            diff -= 2.0 * math.pi
        while diff < -math.pi:
            diff += 2.0 * math.pi
        return diff


def main(args=None):
    rclpy.init(args=args)
    node = WaypointNavigator()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()