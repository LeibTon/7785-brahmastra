#!/usr/bin/env python3
"""
Receives a sign class on /sign_class and executes the required in-place turn.

Sign → action mapping
  0  empty        → do nothing
  1  left         → turn  90° left
  2  right        → turn  90° right
  3  do_not_enter → turn 180° (U-turn)
  4  stop         → turn 180° (U-turn)
  5  goal         → publish /goal_reached True and stop

Publishes:
  /turn_controller/done  (Bool) — True when the turn is complete
  /goal_reached          (Bool) — True when goal sign is seen
"""

import math
import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist
from std_msgs.msg import Bool, Int32


TURN_SPEED = 0.5   # rad/s

TURN_DURATION = {
    1:  math.pi / 2   / TURN_SPEED,          # 90° left
    2:  math.pi / 2   / TURN_SPEED,          # 90° right
    3:  math.pi       / TURN_SPEED,          # 180°
    4:  math.pi       / TURN_SPEED,          # 180°
    6:  math.radians(15) / TURN_SPEED,       # 15° left (spin-search step)
}
TURN_DIRECTION = {
    1:  1.0,   # CCW (+z)
    2: -1.0,   # CW  (-z)
    3:  1.0,
    4:  1.0,
    6:  1.0,   # CCW (+z) — 15° left
}


class TurnController(Node):
    def __init__(self):
        super().__init__('turn_controller')

        self._cmd_pub  = self.create_publisher(Twist, '/cmd_vel', 10)
        self._done_pub = self.create_publisher(Bool, '/turn_controller/done', 10)
        self._goal_pub = self.create_publisher(Bool, '/goal_reached', 10)

        self._sign_sub = self.create_subscription(
            Int32, '/sign_class', self._sign_cb, 10)

        self._turning      = False
        self._turn_cmd     = Twist()
        self._turn_end     = 0.0   # ROS time in seconds
        self._timer = self.create_timer(0.05, self._control_loop)

        self.get_logger().info('TurnController ready')

    def _sign_cb(self, msg: Int32):
        if self._turning:
            return   # ignore new signs while already turning

        sign = msg.data

        if sign == 5:
            self.get_logger().info('Goal sign seen! Stopping.')
            self._cmd_pub.publish(Twist())
            out = Bool(); out.data = True
            self._goal_pub.publish(out)
            self._publish_done()
            return

        if sign not in TURN_DURATION:
            self.get_logger().info(f'Sign {sign}: no turn needed.')
            self._publish_done()
            return

        duration  = TURN_DURATION[sign]
        direction = TURN_DIRECTION[sign]
        self.get_logger().info(
            f'Sign {sign}: turning {"left" if direction > 0 else "right"} '
            f'for {duration:.2f}s')

        self._turn_cmd.angular.z = direction * TURN_SPEED
        self._turn_end = self.get_clock().now().nanoseconds * 1e-9 + duration
        self._turning  = True

    def _control_loop(self):
        if not self._turning:
            return

        now = self.get_clock().now().nanoseconds * 1e-9
        if now < self._turn_end:
            self._cmd_pub.publish(self._turn_cmd)
        else:
            self._cmd_pub.publish(Twist())   # stop
            self._turning = False
            self.get_logger().info('Turn complete.')
            self._publish_done()

    def _publish_done(self):
        out = Bool(); out.data = True
        self._done_pub.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = TurnController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
