#!/usr/bin/env python3

import math
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool, Float32


WALL_STOP_DIST   = 0.5
FRONT_HALF_ANG   = 20.0
LINEAR_SPEED     = 0.2
KP_HEADING       = 1.5
MAX_HEADING_CORR = 0.4


class WallFollower(Node):
    def __init__(self):
        super().__init__('wall_follower')

        sensor_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
        )

        self.cmd_pub        = self.create_publisher(Twist, '/cmd_vel', 10)
        self.wall_reach_pub = self.create_publisher(Bool,  '/wall_reached', 10)

        self.create_subscription(LaserScan, '/scan',                    self.scan_cb,    sensor_qos)
        self.create_subscription(Bool,      '/wall_follower/enable',    self.enable_cb,  10)
        self.create_subscription(Float32,   '/turn_controller/heading', self.heading_cb, 10)
        self.create_subscription(Odometry,  '/odom',                    self.odom_cb,    10)

        self.active         = False
        self.front_dist     = float('inf')
        self.yaw            = 0.0
        self.target_heading = None

        self.create_timer(0.05, self.control_loop)
        self.get_logger().info('WallFollower ready')

    def enable_cb(self, msg: Bool):
        self.active = msg.data
        if not self.active:
            self.publish_cmd(0.0, 0.0)

    def odom_cb(self, msg: Odometry):
        q = msg.pose.pose.orientation
        self.yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                              1.0 - 2.0 * (q.y * q.y + q.z * q.z))

    def heading_cb(self, msg: Float32):
        self.target_heading = msg.data

    def scan_cb(self, msg: LaserScan):
        ranges = []
        for i, r in enumerate(msg.ranges):
            a_deg = math.degrees(msg.angle_min + i * msg.angle_increment)
            while a_deg >  180: a_deg -= 360
            while a_deg < -180: a_deg += 360
            if abs(a_deg) <= FRONT_HALF_ANG and math.isfinite(r) and r > 0.05:
                ranges.append(r)
        self.front_dist = min(ranges) if ranges else float('inf')

    def control_loop(self):
        if not self.active:
            return

        if self.front_dist <= WALL_STOP_DIST:
            self.publish_cmd(0.0, 0.0)
            self.active = False
            m = Bool(); m.data = True
            self.wall_reach_pub.publish(m)
            self.get_logger().info(f'Wall reached at {self.front_dist:.3f} m')
            return

        ang = 0.0
        if self.target_heading is not None:
            err = self.target_heading - self.yaw
            while err >  math.pi: err -= 2 * math.pi
            while err <= -math.pi: err += 2 * math.pi
            ang = max(-MAX_HEADING_CORR, min(MAX_HEADING_CORR, KP_HEADING * err))

        self.publish_cmd(LINEAR_SPEED, ang)

    def publish_cmd(self, linear: float, angular: float):
        t = Twist()
        t.linear.x  = linear
        t.angular.z = angular
        self.cmd_pub.publish(t)


def main(args=None):
    rclpy.init(args=args)
    rclpy.spin(WallFollower())
    rclpy.shutdown()


if __name__ == '__main__':
    main()
