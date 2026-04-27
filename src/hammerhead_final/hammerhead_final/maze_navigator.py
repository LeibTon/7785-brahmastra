#!/usr/bin/env python3

import random
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, Int32

DETECT_SETTLE_S = 1.5
SPIN_SETTLE_S   = 1.2


class MazeNavigator(Node):
    def __init__(self):
        super().__init__('maze_navigator')

        self.drive_pub  = self.create_publisher(Bool,  '/wall_follower/enable', 10)
        self.detect_pub = self.create_publisher(Bool,  '/sign_detector/detect', 10)
        self.sign_pub   = self.create_publisher(Int32, '/sign_class', 10)

        self.create_subscription(Bool,  '/wall_reached',         self.wall_reached_cb, 10)
        self.create_subscription(Int32, '/sign_class',           self.sign_cb,         10)
        self.create_subscription(Bool,  '/turn_controller/done', self.turn_done_cb,    10)
        self.create_subscription(Bool,  '/goal_reached',         self.goal_reached_cb, 10)

        self.state         = 'DRIVE'
        self.spin_attempts = 0
        self.spin_sign     = 6
        self.settle_timer  = None

        self.startup_timer = self.create_timer(2.0, self.start)
        self.get_logger().info('MazeNavigator ready')

    def start(self):
        self.startup_timer.cancel()
        self.drive()

    def drive(self):
        self.state = 'DRIVE'
        self.get_logger().info('[DRIVE]')
        m = Bool(); m.data = True
        self.drive_pub.publish(m)

    def detect(self):
        self.state = 'DETECT'
        self.spin_attempts = 0
        self.spin_sign = random.choice([6, 7])
        self.get_logger().info(f'[DETECT] spin={"CCW" if self.spin_sign == 6 else "CW"}')
        m = Bool(); m.data = True
        self.detect_pub.publish(m)

    def turn(self, sign: int):
        self.state = 'TURN'
        self.get_logger().info(f'[TURN] sign={sign}')
        m = Int32(); m.data = sign
        self.sign_pub.publish(m)

    def wall_reached_cb(self, msg: Bool):
        if msg.data and self.state == 'DRIVE':
            self.state = 'SETTLE'
            self.settle_timer = self.create_timer(DETECT_SETTLE_S, self.settle_done)

    def settle_done(self):
        self.settle_timer.cancel()
        self.settle_timer = None
        self.detect()

    def sign_cb(self, msg: Int32):
        if self.state != 'DETECT':
            return
        sign = msg.data
        if sign == 0:
            self.spin_attempts += 1
            self.get_logger().info(f'Empty sign, spin attempt {self.spin_attempts}')
            if self.spin_attempts > 24:
                self.get_logger().warn('No sign found, driving on.')
                self.drive()
            else:
                self.turn(self.spin_sign)
        else:
            self.turn(sign)

    def turn_done_cb(self, msg: Bool):
        if not msg.data or self.state != 'TURN':
            return
        if self.spin_attempts > 0:
            self.state = 'SETTLE'
            self.settle_timer = self.create_timer(SPIN_SETTLE_S, self.settle_done)
        else:
            self.drive()

    def goal_reached_cb(self, msg: Bool):
        if msg.data:
            self.state = 'DONE'
            self.get_logger().info('=== GOAL REACHED ===')


def main(args=None):
    rclpy.init(args=args)
    rclpy.spin(MazeNavigator())
    rclpy.shutdown()


if __name__ == '__main__':
    main()
