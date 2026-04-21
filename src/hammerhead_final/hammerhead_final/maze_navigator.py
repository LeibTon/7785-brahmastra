#!/usr/bin/env python3
"""
Maze Navigator — state machine.

DRIVE  → drive until wall → DETECT
DETECT → classify sign    → TURN (or SPIN_SEARCH if empty)
TURN   → turn in place    → DRIVE
DONE   → goal reached, stop
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, Int32

DETECT_SETTLE_S = 1.5   # seconds to wait after wall stop before triggering detection
SPIN_SETTLE_S   = 0.5   # seconds to wait after each 15° spin before detecting


class MazeNavigator(Node):
    def __init__(self):
        super().__init__('maze_navigator')

        self._drive_pub   = self.create_publisher(Bool,  '/wall_follower/enable', 10)
        self._detect_pub  = self.create_publisher(Bool,  '/sign_detector/detect', 10)
        self._sign_pub    = self.create_publisher(Int32, '/sign_class', 10)

        self.create_subscription(Bool,  '/wall_reached',         self._wall_reached_cb, 10)
        self.create_subscription(Int32, '/sign_class',           self._sign_cb,         10)
        self.create_subscription(Bool,  '/turn_controller/done', self._turn_done_cb,    10)
        self.create_subscription(Bool,  '/goal_reached',         self._goal_reached_cb, 10)

        self._state         = 'DRIVE'
        self._spin_attempts = 0
        self._settle_timer  = None

        # Wait 2s for other nodes to start, then begin
        self._startup_timer = self.create_timer(2.0, self._start)
        self.get_logger().info('MazeNavigator initialising ...')

    def _start(self):
        self._startup_timer.cancel()
        self._drive()

    def _drive(self):
        self._state = 'DRIVE'
        self.get_logger().info('[DRIVE]')
        msg = Bool(); msg.data = True
        self._drive_pub.publish(msg)

    def _detect(self):
        self._state = 'DETECT'
        self._spin_attempts = 0
        self.get_logger().info('[DETECT]')
        msg = Bool(); msg.data = True
        self._detect_pub.publish(msg)

    def _turn(self, sign: int):
        self._state = 'TURN'
        self.get_logger().info(f'[TURN] sign={sign}')
        msg = Int32(); msg.data = sign
        self._sign_pub.publish(msg)

    # --- callbacks ---

    def _wall_reached_cb(self, msg: Bool):
        if msg.data and self._state == 'DRIVE':
            self._state = 'SETTLE'
            self.get_logger().info('[SETTLE] waiting before detect ...')
            self._settle_timer = self.create_timer(DETECT_SETTLE_S, self._settle_done)

    def _settle_done(self):
        self._settle_timer.cancel()
        self._settle_timer = None
        self._detect()

    def _sign_cb(self, msg: Int32):
        if self._state != 'DETECT':
            return
        sign = msg.data
        if sign == 0:
            self._spin_attempts += 1
            self.get_logger().info(f'Empty sign, spin attempt {self._spin_attempts}')
            if self._spin_attempts > 24:   # 24 × 15° = 360°
                self.get_logger().warn('No sign found, driving on.')
                self._drive()
            else:
                self._turn(6)   # rotate 15° right to look at next wall
        else:
            self._turn(sign)

    def _turn_done_cb(self, msg: Bool):
        if not msg.data:
            return
        if self._state == 'TURN':
            if self._spin_attempts > 0:
                # pause briefly after each spin step before detecting
                self._state = 'SETTLE'
                self.get_logger().info('[SPIN SETTLE] waiting after spin ...')
                self._settle_timer = self.create_timer(SPIN_SETTLE_S, self._settle_done)
            else:
                self._drive()

    def _goal_reached_cb(self, msg: Bool):
        if msg.data:
            self._state = 'DONE'
            self.get_logger().info('=== GOAL REACHED ===')


def main(args=None):
    rclpy.init(args=args)
    rclpy.spin(MazeNavigator())
    rclpy.shutdown()


if __name__ == '__main__':
    main()
