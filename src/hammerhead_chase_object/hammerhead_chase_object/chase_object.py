import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Point
import matplotlib.pyplot as plt
from collections import deque
from datetime import datetime


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

        # Start with P or PD; add I only if you see steady-state error.
        self.declare_parameter('kp_lin', 0.04)
        self.declare_parameter('ki_lin', 0.0)
        self.declare_parameter('kd_lin', 0.005)
        self.declare_parameter('i_lim_lin', 0.5)

        self.declare_parameter('kp_ang', 3.0)
        self.declare_parameter('ki_ang', 0.0)
        self.declare_parameter('kd_ang', 0.1)
        self.declare_parameter('i_lim_ang', 1.0)

        self.declare_parameter('angle_deadband', 0.03)      # rad (~1.7 deg)
        self.declare_parameter('range_deadband', 0.05)      # m
        self.declare_parameter('lost_timeout', 0.5)         # sec
        self.declare_parameter('control_rate', 30.0)        # Hz

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
        
        # Data storage for plotting (received robot state data and commands)
        self.data_obj_angles = deque(maxlen=1000)
        self.data_obj_ranges = deque(maxlen=1000)
        self.data_lin_vel = deque(maxlen=1000)
        self.data_ang_vel = deque(maxlen=1000)
        self.data_ang_err = deque(maxlen=1000)
        self.data_rng_err = deque(maxlen=1000)
        self.data_timestamps = deque(maxlen=1000)
        self.start_time = datetime.now()

    def obj_cb(self, msg: Point):
        self.obj_angle = float(msg.x)
        self.obj_range = float(msg.y)
        self.obj_valid = (msg.z >= 0.5) and math.isfinite(self.obj_angle) and math.isfinite(self.obj_range)
        self.last_msg_time = self.get_clock().now()
        
        # Store received data for plotting
        elapsed = (datetime.now() - self.start_time).total_seconds()
        self.data_obj_angles.append(self.obj_angle)
        self.data_obj_ranges.append(self.obj_range)
        self.data_timestamps.append(elapsed)

    def stop_robot(self):
        t = Twist()
        self.cmd_pub.publish(t)
    
    def plot_data(self):
        """Plot received robot_state data and sent command data."""
        if not self.data_timestamps:
            self.get_logger().info('No data to plot')
            return
        
        plt.figure(figsize=(16, 10))
        
        # Plot received object angle
        plt.subplot(2, 3, 1)
        plt.plot(self.data_timestamps, self.data_obj_angles, 'b-', label='Object Angle')
        plt.axhline(y=0, color='r', linestyle='--', alpha=0.5, label='Zero')
        plt.xlabel('Time (s)')
        plt.ylabel('Angle (rad)')
        plt.title('Received Object Angle Over Time')
        plt.grid(True)
        plt.legend()
        
        # Plot received object range
        plt.subplot(2, 3, 2)
        plt.plot(self.data_timestamps, self.data_obj_ranges, 'g-', label='Object Range')
        plt.axhline(y=self.desired_distance, color='r', linestyle='--', label=f'Desired ({self.desired_distance:.2f}m)')
        plt.xlabel('Time (s)')
        plt.ylabel('Range (m)')
        plt.title('Received Object Range Over Time')
        plt.grid(True)
        plt.legend()
        
        # Plot linear velocity command
        plt.subplot(2, 3, 3)
        plt.plot(self.data_timestamps, self.data_lin_vel, 'b-', label='Linear Velocity')
        plt.axhline(y=0, color='k', linestyle='-', alpha=0.3)
        plt.xlabel('Time (s)')
        plt.ylabel('Velocity (m/s)')
        plt.title('Linear Velocity Command Over Time')
        plt.grid(True)
        plt.legend()
        
        # Plot angular velocity command
        plt.subplot(2, 3, 4)
        plt.plot(self.data_timestamps, self.data_ang_vel, 'r-', label='Angular Velocity')
        plt.axhline(y=0, color='k', linestyle='-', alpha=0.3)
        plt.xlabel('Time (s)')
        plt.ylabel('Velocity (rad/s)')
        plt.title('Angular Velocity Command Over Time')
        plt.grid(True)
        plt.legend()
        
        # Plot angle and range errors
        plt.subplot(2, 3, 5)
        ax1 = plt.gca()
        ax1.plot(self.data_timestamps, self.data_ang_err, 'g-', label='Angle Error', alpha=0.7)
        ax1.set_xlabel('Time (s)')
        ax1.set_ylabel('Angle Error (rad)', color='g')
        ax1.tick_params(axis='y', labelcolor='g')
        ax1.grid(True)
        
        ax2 = ax1.twinx()
        ax2.plot(self.data_timestamps, self.data_rng_err, 'm-', label='Range Error', alpha=0.7)
        ax2.set_ylabel('Range Error (m)', color='m')
        ax2.tick_params(axis='y', labelcolor='m')
        
        plt.title('Both Errors Over Time')
        ax1.legend(loc='upper left')
        ax2.legend(loc='upper right')
        
        # Phase plot: angular velocity vs linear velocity
        plt.subplot(2, 3, 6)
        plt.plot(self.data_ang_vel, self.data_lin_vel, 'c-', alpha=0.7, label='Phase')
        if len(self.data_ang_vel) > 0:
            plt.scatter(self.data_ang_vel[-1], self.data_lin_vel[-1], color='r', s=100, label='End', zorder=5)
        plt.xlabel('Angular Velocity (rad/s)')
        plt.ylabel('Linear Velocity (m/s)')
        plt.title('Phase Plot: Angular vs Linear Velocity')
        plt.grid(True)
        plt.legend()
        
        plt.tight_layout()
        filename = f'chase_object_plot_{datetime.now().strftime("%Y%m%d_%H%M%S")}.png'
        plt.savefig(filename)
        self.get_logger().info(f'Plot saved to {filename}')
        plt.show()

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

        # Control (note sign: if your robot turns the wrong way, flip angular sign)
        u_ang = self.pid_ang.update(ang_err, dt)
        u_lin = self.pid_lin.update(rng_err, dt)

        twist = Twist()
        twist.angular.z = clamp(u_ang, -self.max_ang, self.max_ang)
        twist.linear.x = clamp(u_lin, -self.max_lin, self.max_lin)

        self.get_logger().info(f'Control: ang_err={ang_err:.3f} rad, rng_err={rng_err:.3f} m -> cmd_lin={u_lin:.3f} m/s, cmd_ang={twist.angular.z:.3f} rad/s')

        # Store control data for plotting
        elapsed = (datetime.now() - self.start_time).total_seconds()
        self.data_lin_vel.append(twist.linear.x)
        self.data_ang_vel.append(twist.angular.z)
        self.data_ang_err.append(ang_err)
        self.data_rng_err.append(rng_err)

        self.cmd_pub.publish(twist)


def main(args=None):
    rclpy.init(args=args)
    node = ChaseObject()
    
    def shutdown_handler(signum, frame):
        print('\nShutting down and generating plots...')
        node.plot_data()
        node.destroy_node()
        rclpy.shutdown()
        exit(0)
    
    import signal
    signal.signal(signal.SIGINT, shutdown_handler)
    
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()