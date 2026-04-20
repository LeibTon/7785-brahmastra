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


WALL_STOP_DIST = 0.45   # metres — stop when wall is this close
FRONT_HALF_ANG = 20.0   # degrees — cone in front to check
LINEAR_SPEED   = 0.12   # m/s forward speed



KP_CENTER = 1      # Proportional gain for centering (tuned down for smoother turns)
KD_CENTER = 0.3     # Derivative gain for centering (tune as needed)
SIDE_ANG = 45.0      # Degrees for left/right wall detection
SIDE_WIDTH = 10.0    # +/- degrees window for averaging side distances
MAX_WALL_DIST = 2.0  # Max distance to consider a wall present
MAX_VALID_SIDE_DIST = 0.7  # Ignore side walls farther than this (meters)

def clamp(x, lo, hi):
    return max(lo, min(hi, x))


class WallCentering(Node):
    def __init__(self):
        super().__init__('wall_centering')

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
        self._left_dist  = float('inf')
        self._right_dist = float('inf')

        self._enable_sub = self.create_subscription(
            Bool, '/wall_follower/enable', self._enable_cb, 10)

        self._prev_error = 0.0
        self._prev_time = self.get_clock().now()

        self._timer = self.create_timer(0.05, self._control_loop)
        self.get_logger().info('WallCentering ready')

    # ------------------------------------------------------------------ #
    def _enable_cb(self, msg: Bool):
        self._active = msg.data
        if not self._active:
            self._publish_cmd(0.0, 0.0)

    def _scan_cb(self, msg: LaserScan):
        front_ranges = []
        left_ranges = []
        right_ranges = []
        # Define sector windows in radians
        front_min = math.radians(-FRONT_HALF_ANG)
        front_max = math.radians(FRONT_HALF_ANG)
        left_min = math.radians(90 - SIDE_WIDTH)
        left_max = math.radians(90 + SIDE_WIDTH)
        right_min = math.radians(-90 - SIDE_WIDTH)
        right_max = math.radians(-90 + SIDE_WIDTH)

        def in_sector(angle, sector_min, sector_max):
            angle = (angle + 2*math.pi) % (2*math.pi)
            sector_min = (sector_min + 2*math.pi) % (2*math.pi)
            sector_max = (sector_max + 2*math.pi) % (2*math.pi)
            if sector_min <= sector_max:
                return sector_min <= angle <= sector_max
            return angle >= sector_min or angle <= sector_max

        for i, r in enumerate(msg.ranges):
            angle = msg.angle_min + i * msg.angle_increment
            if not (math.isfinite(r) and r > 0.05):
                continue
            # Front sector
            if in_sector(angle, front_min, front_max):
                front_ranges.append(r)
            # Left sector (90 deg)
            if in_sector(angle, left_min, left_max):
                left_ranges.append(r)
            # Right sector (-90 deg)
            if in_sector(angle, right_min, right_max):
                right_ranges.append(r)

        self._front_dist = min(front_ranges) if front_ranges else float('inf')
        self._left_dist  = sum(left_ranges)/len(left_ranges) if left_ranges else float('inf')
        self._right_dist = sum(right_ranges)/len(right_ranges) if right_ranges else float('inf')

        # Ignore side walls that are too far away
        if self._left_dist > MAX_VALID_SIDE_DIST:
            self._left_dist = float('inf')
        if self._right_dist > MAX_VALID_SIDE_DIST:
            self._right_dist = float('inf')

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

        # --- Centering logic ---
        left_valid = self._left_dist < MAX_WALL_DIST
        right_valid = self._right_dist < MAX_WALL_DIST

        error = 0.0
        DESIRED_DIST = 0.45
        if left_valid and right_valid:
            # Center between both walls
            error = self._right_dist - self._left_dist
            self.get_logger().info(f'Centering: left={self._left_dist:.2f}m right={self._right_dist:.2f}m error={error:.2f}')
        elif left_valid:
            # Follow left wall
            error = DESIRED_DIST - self._left_dist
            self.get_logger().info(f'Left wall only: left={self._left_dist:.2f}m error={error:.2f}')
        elif right_valid:
            # Follow right wall
            error = self._right_dist - DESIRED_DIST
            self.get_logger().info(f'Right wall only: right={self._right_dist:.2f}m error={error:.2f}')
        else:
            # No walls detected, go straight
            error = 0.0
            self.get_logger().info('No side walls detected, driving straight')

        # PD controller
        now = self.get_clock().now()
        dt = (now - self._prev_time).nanoseconds * 1e-9
        derivative = 0.0
        if dt > 0.0:
            derivative = (error - self._prev_error) / dt
        self._prev_error = error
        self._prev_time = now

        ang = clamp(-KP_CENTER * error - KD_CENTER * derivative, -0.5, 0.5)
        self._publish_cmd(LINEAR_SPEED, ang)

    def _publish_cmd(self, linear: float, angular: float):
        t = Twist()
        t.linear.x  = linear
        t.angular.z = angular
        self._cmd_pub.publish(t)



def main(args=None):
    rclpy.init(args=args)
    node = WallCentering()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
