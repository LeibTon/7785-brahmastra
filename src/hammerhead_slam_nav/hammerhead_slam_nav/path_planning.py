import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import FollowWaypoints


class Nav2ActionClient(Node):
    def __init__(self):
        super().__init__('nav2_action_client')
        self.action_client = ActionClient(self, FollowWaypoints, 'follow_waypoints')

        # Waypoints are position-only; orientation is fixed to (0, 0, 0, 1).
        self.waypoints = [
            (1.3991, 0.6282),
            (0.4570, 1.6008),
            (1.9889, 0.7515),
        ]
        self.current_waypoint_index = 0

        self.get_logger().info('Waiting for Nav2 FollowWaypoints action server...')
        self.action_client.wait_for_server()
        self.get_logger().info('Action server available. Starting waypoint navigation.')

    def send_next_waypoint(self):
        if self.current_waypoint_index >= len(self.waypoints):
            self.get_logger().info('All waypoints completed successfully.')
            return

        x, y = self.waypoints[self.current_waypoint_index]

        goal_msg = FollowWaypoints.Goal()
        waypoint = PoseStamped()
        waypoint.header.frame_id = 'map'
        waypoint.header.stamp = self.get_clock().now().to_msg()
        waypoint.pose.position.x = x
        waypoint.pose.position.y = y
        waypoint.pose.orientation.x = 0.0
        waypoint.pose.orientation.y = 0.0
        waypoint.pose.orientation.z = 0.0
        waypoint.pose.orientation.w = 1.0
        goal_msg.poses = [waypoint]

        self.get_logger().info(
            f'Sending waypoint {self.current_waypoint_index + 1}/{len(self.waypoints)}: '
            f'x={x:.2f}, y={y:.2f}'
        )

        send_goal_future = self.action_client.send_goal_async(
            goal_msg,
            feedback_callback=self.feedback_callback,
        )
        send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error(
                f'Waypoint {self.current_waypoint_index + 1} was rejected by Nav2.'
            )
            return

        self.get_logger().info(
            f'Waypoint {self.current_waypoint_index + 1} accepted. Waiting for result...'
        )
        get_result_future = goal_handle.get_result_async()
        get_result_future.add_done_callback(self.result_callback)

    def result_callback(self, future):
        result = future.result()
        status = result.status
        missed_waypoints = result.result.missed_waypoints

        if status == GoalStatus.STATUS_SUCCEEDED and len(missed_waypoints) == 0:
            self.get_logger().info(
                f'Waypoint {self.current_waypoint_index + 1} reached successfully.'
            )
            self.current_waypoint_index += 1
            self.send_next_waypoint()
            return

        if status == GoalStatus.STATUS_SUCCEEDED and len(missed_waypoints) > 0:
            self.get_logger().error(
                f'Waypoint {self.current_waypoint_index + 1} reported as missed by Nav2: '
                f'{missed_waypoints}'
            )
            return

        self.get_logger().error(
            f'Waypoint {self.current_waypoint_index + 1} failed with status code {status}.'
        )

    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        self.get_logger().info(
            f'Feedback for waypoint {self.current_waypoint_index + 1}: '
            f'current waypoint index in server = {feedback.current_waypoint}'
        )


def main(args=None):
    rclpy.init(args=args)
    node = Nav2ActionClient()
    node.send_next_waypoint()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()