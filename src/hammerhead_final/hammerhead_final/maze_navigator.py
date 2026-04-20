#!/usr/bin/env python3
"""
Maze Navigator — top-level state machine.

States
------
DRIVE       Drive straight until a wall is close (WallFollower signals /wall_reached).
DETECT      Trigger sign classification (SignDetector) and wait for /sign_class.
TURN        Execute the required turn (TurnController) and wait for /turn_controller/done.
SPIN_SEARCH Robot couldn't read a sign (got class 0 = empty); rotate 90° and retry.
DONE        Goal reached — stop everything.

Topic wiring (this node controls the sub-nodes via enable/trigger topics):
  Publishes:
    /wall_follower/enable      Bool  — start/stop driving
    /sign_detector/detect      Bool  — trigger one classification
  Subscribes:
    /wall_reached              Bool  — wall follower finished
    /sign_class                Int32 — classification result
    /turn_controller/done      Bool  — turn finished
    /goal_reached              Bool  — goal sign seen
"""

import rclpy
from rclpy.node import Node

from std_msgs.msg import Bool, Int32


class State:
    DRIVE       = 'DRIVE'
    DETECT      = 'DETECT'
    TURN        = 'TURN'
    SPIN_SEARCH = 'SPIN_SEARCH'
    DONE        = 'DONE'


MAX_SPIN_ATTEMPTS = 4   # try 4 × 90° before giving up


class MazeNavigator(Node):
    def __init__(self):
        super().__init__('maze_navigator')

        # Publishers to control sub-nodes
        self._drive_pub   = self.create_publisher(Bool, '/wall_follower/enable', 10)
        self._detect_pub  = self.create_publisher(Bool, '/sign_detector/detect', 10)

        # Subscriptions from sub-nodes
        self.create_subscription(Bool,  '/wall_reached',          self._wall_reached_cb,  10)
        self.create_subscription(Int32, '/sign_class',            self._sign_class_cb,    10)
        self.create_subscription(Bool,  '/turn_controller/done',  self._turn_done_cb,     10)
        self.create_subscription(Bool,  '/goal_reached',          self._goal_reached_cb,  10)

        self._state         = State.DRIVE
        self._spin_attempts = 0
        self._pending_sign  = None   # remember sign while turn is executing

        # Small delay so sub-nodes finish spinning up before we start
        self._startup_timer = self.create_timer(2.0, self._start)
        self.get_logger().info('MazeNavigator initialising …')

    # ------------------------------------------------------------------ #
    # Startup
    # ------------------------------------------------------------------ #
    def _start(self):
        self._startup_timer.cancel()
        self.get_logger().info('Starting maze run — entering DRIVE state')
        self._enter_drive()

    # ------------------------------------------------------------------ #
    # State transitions
    # ------------------------------------------------------------------ #
    def _enter_drive(self):
        self._state = State.DRIVE
        self.get_logger().info('[DRIVE] Moving toward wall …')
        msg = Bool(); msg.data = True
        self._drive_pub.publish(msg)

    def _enter_detect(self):
        self._state = State.DETECT
        self._spin_attempts = 0
        self.get_logger().info('[DETECT] Requesting sign classification …')
        msg = Bool(); msg.data = True
        self._detect_pub.publish(msg)

    def _enter_turn(self, sign_class: int):
        self._state = State.TURN
        self._pending_sign = sign_class
        self.get_logger().info(f'[TURN] Executing action for sign {sign_class}')
        # TurnController already subscribed to /sign_class — re-publish so it sees it
        out = Int32(); out.data = sign_class
        # We publish directly on /sign_class — TurnController listens there
        # (navigator does NOT own that topic; sign_detector does, but we forward here)
        self._sign_forward_pub.publish(out)

    def _enter_spin_search(self):
        self._state = State.SPIN_SEARCH
        self._spin_attempts += 1
        self.get_logger().info(
            f'[SPIN_SEARCH] Empty sign — attempt {self._spin_attempts}/{MAX_SPIN_ATTEMPTS}')
        if self._spin_attempts > MAX_SPIN_ATTEMPTS:
            self.get_logger().warn('Could not find a sign — driving on anyway.')
            self._enter_drive()
            return
        # Ask TurnController to rotate 90° right (sign class 2) to look at another wall
        out = Int32(); out.data = 2
        self._sign_forward_pub.publish(out)

    # ------------------------------------------------------------------ #
    # Callbacks
    # ------------------------------------------------------------------ #
    def _wall_reached_cb(self, msg: Bool):
        if msg.data and self._state == State.DRIVE:
            self._enter_detect()

    def _sign_class_cb(self, msg: Int32):
        if self._state != State.DETECT:
            return
        sign = msg.data
        if sign == 0:
            self._enter_spin_search()
        else:
            self._enter_turn(sign)

    def _turn_done_cb(self, msg: Bool):
        if not msg.data:
            return
        if self._state in (State.TURN, State.SPIN_SEARCH):
            if self._state == State.SPIN_SEARCH:
                # After spin, try detecting again
                self._enter_detect()
            else:
                # After a real directional turn, drive to next wall
                self._enter_drive()

    def _goal_reached_cb(self, msg: Bool):
        if msg.data:
            self._state = State.DONE
            self.get_logger().info('=== GOAL REACHED — maze complete! ===')


def main(args=None):
    rclpy.init(args=args)
    node = MazeNavigator()
    # Patch: add the forwarding publisher after __init__ so TurnController receives signs
    node._sign_forward_pub = node.create_publisher(Int32, '/sign_class', 10)
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
