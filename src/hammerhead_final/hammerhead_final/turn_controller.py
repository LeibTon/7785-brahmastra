#!/usr/bin/env python3
"""
Receives a sign class on /sign_class, first aligns perpendicular to the wall
using LIDAR, then executes the required turn.

Sign → turn after alignment
  1  left         →  +90°
  2  right        →  -90°
  3  do_not_enter → +180°
  4  stop         → +180°
  5  goal         → stop, publish /goal_reached

Publishes:
  /turn_controller/done  (Bool)
  /goal_reached          (Bool)
"""

import math
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy

from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool, Int32


TURN_SPEED      = 0.5    # rad/s — speed for sign turn
ALIGN_SPEED     = 0.3    # rad/s — speed for wall alignment
ALIGN_HALF_ANG  = 30.0   # deg   — LIDAR cone to find wall angle
ALIGN_TOLERANCE = math.radians(3.0)   # 3°

TURN_ANGLE = {
    1:  math.pi / 2,   # left  90°
    2: -math.pi / 2,   # right 90°
    3:  math.pi,       # U-turn
    4:  math.pi,       # U-turn
    6: -math.pi / 12,  # 15° right (spin-search)
}


class TurnController(Node):
    def __init__(self):
        super().__init__('turn_controller')

        sensor_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
        )

        self._cmd_pub  = self.create_publisher(Twist, '/cmd_vel', 10)
        self._done_pub = self.create_publisher(Bool, '/turn_controller/done', 10)
        self._goal_pub = self.create_publisher(Bool, '/goal_reached', 10)

        self.create_subscription(Int32,   '/sign_class', self._sign_cb,  10)
        self.create_subscription(LaserScan, '/scan',     self._scan_cb,  sensor_qos)

        self._wall_angle  = 0.0    # angle offset to nearest wall (rad), 0 = square-on
        self._state       = 'IDLE' # IDLE → ALIGN → TURN
        self._sign        = None
        self._turn_end    = 0.0

        self._timer = self.create_timer(0.05, self._control_loop)
        self.get_logger().info('TurnController ready')

    # ------------------------------------------------------------------ #
    def _scan_cb(self, msg: LaserScan):
        wide_angles = []
        wide_ranges = []
        for i, r in enumerate(msg.ranges):
            a_deg = math.degrees(msg.angle_min + i * msg.angle_increment)
            while a_deg >  180: a_deg -= 360
            while a_deg < -180: a_deg += 360
            if abs(a_deg) <= ALIGN_HALF_ANG and math.isfinite(r) and r > 0.05:
                wide_angles.append(math.radians(a_deg))
                wide_ranges.append(r)

        if wide_ranges:
            idx = wide_ranges.index(min(wide_ranges))
            self._wall_angle = wide_angles[idx]

    def _sign_cb(self, msg: Int32):
        if self._state != 'IDLE':
            return

        sign = msg.data

        if sign == 5:
            self.get_logger().info('Goal!')
            self._cmd_pub.publish(Twist())
            out = Bool(); out.data = True
            self._goal_pub.publish(out)
            self._publish_done()
            return

        if sign not in TURN_ANGLE:
            self.get_logger().info(f'Sign {sign}: no turn.')
            self._publish_done()
            return

        self._sign  = sign
        self._state = 'ALIGN'
        self.get_logger().info(
            f'Sign {sign}: aligning to wall first (offset {math.degrees(self._wall_angle):.1f}°)')

    def _control_loop(self):
        if self._state == 'IDLE':
            return

        if self._state == 'ALIGN':
            if abs(self._wall_angle) <= ALIGN_TOLERANCE:
                # Aligned — now start the sign turn
                self._cmd_pub.publish(Twist())
                angle    = TURN_ANGLE[self._sign]
                duration = abs(angle) / TURN_SPEED
                self._turn_dir = 1.0 if angle > 0 else -1.0
                self._turn_end = self.get_clock().now().nanoseconds * 1e-9 + duration
                self._state    = 'TURN'
                self.get_logger().info(
                    f'Aligned. Turning {"left" if angle > 0 else "right"} '
                    f'{math.degrees(abs(angle)):.0f}° for {duration:.2f}s')
            else:
                direction = -1.0 if self._wall_angle > 0 else 1.0
                t = Twist(); t.angular.z = direction * ALIGN_SPEED
                self._cmd_pub.publish(t)

        elif self._state == 'TURN':
            now = self.get_clock().now().nanoseconds * 1e-9
            if now < self._turn_end:
                t = Twist(); t.angular.z = self._turn_dir * TURN_SPEED
                self._cmd_pub.publish(t)
            else:
                self._cmd_pub.publish(Twist())
                self._state = 'IDLE'
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
