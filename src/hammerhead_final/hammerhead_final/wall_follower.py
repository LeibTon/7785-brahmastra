#!/usr/bin/env python3
"""
Drive straight until the LIDAR detects a wall within a threshold distance.
Publishes /wall_reached (Bool) when stopped at a wall.
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy

from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool

import math


WALL_STOP_DIST = 0.7   # metres — stop when wall is this close
FRONT_HALF_ANG = 20.0   # degrees — cone in front to check
LINEAR_SPEED   = 0.12   # m/s forward speed


class WallFollower(Node):
    def __init__(self):
        super().__init__('wall_follower')

        sensor_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
        )

        self._cmd_pub        = self.create_publisher(Twist, '/cmd_vel', 10)
        self._wall_reach_pub = self.create_publisher(Bool, '/wall_reached', 10)
        self._scan_sub       = self.create_subscription(
            LaserScan, '/scan', self._scan_cb, sensor_qos)

        self._active     = False
        self._front_dist = float('inf')

        self._enable_sub = self.create_subscription(
            Bool, '/wall_follower/enable', self._enable_cb, 10)

        self._timer = self.create_timer(0.05, self._control_loop)
        self.get_logger().info('WallFollower ready')

    # ------------------------------------------------------------------ #
    def _enable_cb(self, msg: Bool):
        self._active = msg.data
        if not self._active:
            self._publish_cmd(0.0, 0.0)

    def _scan_cb(self, msg: LaserScan):
        ranges = []
        for i, r in enumerate(msg.ranges):
            a_deg = math.degrees(msg.angle_min + i * msg.angle_increment)
            while a_deg >  180: a_deg -= 360
            while a_deg < -180: a_deg += 360
            if abs(a_deg) <= FRONT_HALF_ANG and math.isfinite(r) and r > 0.05:
                ranges.append(r)

        if ranges:
            self._front_dist = min(ranges)
        else:
            self._front_dist = float('inf')

    def _control_loop(self):
        if not self._active:
            return

        if self._front_dist <= WALL_STOP_DIST:
            self._publish_cmd(0.0, 0.0)
            self._active = False
            reached = Bool()
            reached.data = True
            self._wall_reach_pub.publish(reached)
            self.get_logger().info(
                f'Wall reached at {self._front_dist:.3f} m')
            return

        self._publish_cmd(LINEAR_SPEED, 0.0)

    def _publish_cmd(self, linear: float, angular: float):
        t = Twist()
        t.linear.x  = linear
        t.angular.z = angular
        self._cmd_pub.publish(t)


def main(args=None):
    rclpy.init(args=args)
    node = WallFollower()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
