#!/usr/bin/env python3

import math
import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool, Float32, Int32


TURN_SPEED = 0.4
KP_TURN    = 1.2
ANGLE_TOL  = math.radians(1.5)
ALIGN_TOL  = math.radians(2.0)

TURN_ANGLE = {
    1:  math.pi / 2,
    2: -math.pi / 2,
    3:  math.pi,
    4:  math.pi,
    6:  math.radians(15),
    7: -math.radians(15),
}

SKIP_ALIGN = {6, 7}

IDLE  = 0
ALIGN = 1
TURN  = 2


def yaw_from_odom(msg: Odometry) -> float:
    q = msg.pose.pose.orientation
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def normalize(a: float) -> float:
    while a >  math.pi: a -= 2 * math.pi
    while a <= -math.pi: a += 2 * math.pi
    return a


def snap_cardinal(yaw: float, yaw_init: float) -> float:
    quarter = math.pi / 2
    snapped = round(normalize(yaw - yaw_init) / quarter) * quarter
    return normalize(yaw_init + snapped)


class TurnController(Node):
    def __init__(self):
        super().__init__('turn_controller')

        self.cmd_pub     = self.create_publisher(Twist,   '/cmd_vel', 10)
        self.done_pub    = self.create_publisher(Bool,    '/turn_controller/done', 10)
        self.goal_pub    = self.create_publisher(Bool,    '/goal_reached', 10)
        self.heading_pub = self.create_publisher(Float32, '/turn_controller/heading', 10)

        self.create_subscription(Int32,    '/sign_class', self.sign_cb, 10)
        self.create_subscription(Odometry, '/odom',       self.odom_cb, 10)

        self.yaw            = 0.0
        self.yaw_init       = 0.0
        self.yaw_ready      = False
        self.target_heading = 0.0
        self.phase          = IDLE
        self.pending_sign   = 0
        self.queued_sign    = None
        self.align_target   = 0.0
        self.turn_target    = 0.0

        self.create_timer(0.05, self.control_loop)
        self.get_logger().info('TurnController ready')

    def odom_cb(self, msg: Odometry):
        self.yaw = yaw_from_odom(msg)
        if not self.yaw_ready:
            self.yaw_init       = self.yaw
            self.target_heading = self.yaw
            self.yaw_ready      = True
            self.publish_heading()
            if self.queued_sign is not None:
                queued = self.queued_sign
                self.queued_sign = None
                self.process_sign(queued)

    def sign_cb(self, msg: Int32):
        self.process_sign(msg.data)

    def process_sign(self, sign: int):
        if self.phase != IDLE:
            return

        if sign == 5:
            self.cmd_pub.publish(Twist())
            out = Bool(); out.data = True
            self.goal_pub.publish(out)
            self.publish_done()
            return

        if sign not in TURN_ANGLE:
            self.publish_done()
            return

        if not self.yaw_ready:
            self.queued_sign = sign
            return

        if sign in SKIP_ALIGN:
            self.align_target = self.yaw
            self.start_turn_phase(sign)
            return

        self.align_target = snap_cardinal(self.yaw, self.yaw_init)
        align_err = normalize(self.align_target - self.yaw)

        if abs(align_err) < ALIGN_TOL:
            self.start_turn_phase(sign)
        else:
            self.pending_sign = sign
            self.phase = ALIGN

    def control_loop(self):
        if self.phase == IDLE or not self.yaw_ready:
            return

        if self.phase == ALIGN:
            err = normalize(self.align_target - self.yaw)
            if abs(err) < ALIGN_TOL:
                self.cmd_pub.publish(Twist())
                self.start_turn_phase(self.pending_sign)
            else:
                self.spin_toward(err)

        elif self.phase == TURN:
            err = normalize(self.turn_target - self.yaw)
            if abs(err) < ANGLE_TOL:
                self.cmd_pub.publish(Twist())
                self.phase          = IDLE
                self.target_heading = self.turn_target
                self.publish_heading()
                self.publish_done()
            else:
                self.spin_toward(err)

    def spin_toward(self, error: float):
        speed = max(-TURN_SPEED, min(TURN_SPEED, KP_TURN * error))
        if abs(speed) < 0.08 and abs(error) > ANGLE_TOL:
            speed = math.copysign(0.08, speed)
        t = Twist()
        t.angular.z = speed
        self.cmd_pub.publish(t)

    def start_turn_phase(self, sign: int):
        self.turn_target = normalize(self.align_target + TURN_ANGLE[sign])
        self.phase = TURN

    def publish_heading(self):
        m = Float32(); m.data = float(self.target_heading)
        self.heading_pub.publish(m)

    def publish_done(self):
        m = Bool(); m.data = True
        self.done_pub.publish(m)


def main(args=None):
    rclpy.init(args=args)
    rclpy.spin(TurnController())
    rclpy.shutdown()


if __name__ == '__main__':
    main()
