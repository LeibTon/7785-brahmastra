import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Point


def clamp(x, lo, hi):
    return max(lo, min(hi, x))


class PID:
    def __init__(self, kp, ki, kd, i_limit):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.i_limit = abs(i_limit)

        self.i = 0.0
        self.prev_e = None

    def reset(self):
        self.i = 0.0
        self.prev_e = None

    def update(self, e, dt):
        if dt <= 0.0:
            return 0.0

        # Integral with anti-windup clamp
        self.i += e * dt
        self.i = clamp(self.i, -self.i_limit, self.i_limit)

        # Derivative
        de = 0.0 if self.prev_e is None else (e - self.prev_e) / dt
        self.prev_e = e

        return self.kp * e + self.ki * self.i + self.kd * de


class ChaseObject(Node):
    def __init__(self):
        super().__init__('chase_object')

        # ---- Tunable params ----
        self.declare_parameter('desired_distance', 0.4)     # meters
        self.declare_parameter('max_lin', 0.15)             # 0.15  m/s (Burger safe-ish) Starting with zero so we can fine tune angle PID first
        self.declare_parameter('max_ang', 1.2)              # rad/s

        # Start with P or PD
        self.declare_parameter('kp_lin', 0.5)
        self.declare_parameter('ki_lin', 0.0)
        self.declare_parameter('kd_lin', 0.1)
        self.declare_parameter('i_lim_lin', 0.5)

        self.declare_parameter('kp_ang', 3.0)
        self.declare_parameter('ki_ang', 0.0)
        self.declare_parameter('kd_ang', 0.1)
        self.declare_parameter('i_lim_ang', 1.0)

        self.declare_parameter('angle_deadband', 0.06)      # rad (~1.7 deg)
        self.declare_parameter('range_deadband', 0.05)      # m
        self.declare_parameter('lost_timeout', 0.5)         # sec
        self.declare_parameter('control_rate', 100.0)        # Hz

        self.desired_distance = float(self.get_parameter('desired_distance').value)
        self.max_lin = float(self.get_parameter('max_lin').value)
        self.max_ang = float(self.get_parameter('max_ang').value)

        self.pid_lin = PID(
            float(self.get_parameter('kp_lin').value),
            float(self.get_parameter('ki_lin').value),
            float(self.get_parameter('kd_lin').value),
            float(self.get_parameter('i_lim_lin').value),
        )
        self.pid_ang = PID(
            float(self.get_parameter('kp_ang').value),
            float(self.get_parameter('ki_ang').value),
            float(self.get_parameter('kd_ang').value),
            float(self.get_parameter('i_lim_ang').value),
        )

        self.angle_deadband = float(self.get_parameter('angle_deadband').value)
        self.range_deadband = float(self.get_parameter('range_deadband').value)
        self.lost_timeout = float(self.get_parameter('lost_timeout').value)

        rate = float(self.get_parameter('control_rate').value)
        self.dt_cmd = 1.0 / rate

        # ---- I/O ----
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.obj_sub = self.create_subscription(Point, '/robot_state', self.obj_cb, 10)

        # ---- State ----
        self.last_msg_time = self.get_clock().now()
        self.obj_angle = 0.0
        self.obj_range = float('inf')
        self.obj_valid = False

        self.last_control_time = self.get_clock().now()
        self.timer = self.create_timer(self.dt_cmd, self.control_loop)

        self.get_logger().info('ChaseObject running: sub /robot_state (Point), pub /cmd_vel')

    def obj_cb(self, msg: Point):
        self.obj_angle = float(msg.x)
        self.obj_range = float(msg.y)
        self.obj_valid = (msg.z >= 0.5) and math.isfinite(self.obj_angle) and math.isfinite(self.obj_range)
        self.last_msg_time = self.get_clock().now()

    def stop_robot(self):
        t = Twist()
        self.cmd_pub.publish(t)

    def control_loop(self):
        now = self.get_clock().now()
        dt = (now - self.last_control_time).nanoseconds * 1e-9
        self.last_control_time = now

        # Stop if object info is stale or invalid
        age = (now - self.last_msg_time).nanoseconds * 1e-9
        if (not self.obj_valid) or (age > self.lost_timeout):
            self.pid_lin.reset()
            self.pid_ang.reset()
            self.stop_robot()
            return

        # Errors
        ang_err = self.obj_angle                          # want angle -> 0
        rng_err = (self.obj_range - self.desired_distance)  # want range -> desired

        # Deadbands to prevent jitter
        if abs(ang_err) < self.angle_deadband:
            ang_err = 0.0
        if abs(rng_err) < self.range_deadband:
            rng_err = 0.0
        # PID control
        u_ang = self.pid_ang.update(ang_err, dt)
        u_lin = self.pid_lin.update(rng_err, dt)

        twist = Twist()
        twist.angular.z = clamp(u_ang, -self.max_ang, self.max_ang)
        twist.linear.x = clamp(u_lin, -self.max_lin, self.max_lin)

        self.get_logger().info(f'Control: ang_err={ang_err:.3f} rad, rng_err={rng_err:.3f} m -> cmd_lin={u_lin:.3f} m/s, cmd_ang={twist.angular.z:.3f} rad/s')

        self.cmd_pub.publish(twist)


def main(args=None):
    rclpy.init(args=args)
    node = ChaseObject()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()