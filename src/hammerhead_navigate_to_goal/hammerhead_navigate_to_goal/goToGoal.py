import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point, Twist
from nav_msgs.msg import Odometry
from rclpy.qos import qos_profile_sensor_data
import math
import select
import sys
import termios
import tty
import numpy as np

class GoToGoal(Node):
    def __init__(self):
        super().__init__('go_to_goal')
        self.Init = True
        self.Init_pos = Point()
        self.Init_pos.x = 0.0
        self.Init_pos.y = 0.0
        self.Init_ang = 0.0

        self.globalPos = Point()
        self.globalAng = 0.0

        ## Obstacle Info
        self.obstaclePos = Point()
        self.obstaclePosGlobal = Point()
        self.has_obstacle = False

        self.odom_sub = self.create_subscription(
            Odometry, '/odom', self.odom_callback, 10
        )
        self.object_sub = self.create_subscription(
            Point, '/object_location', self.object_callback, 10
        )
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.timer = self.create_timer(0.1, self.control_loop)
        self.waypoints = [[1.5, 0.0], [1.5, 1.4], [0.0, 1.4]]
        self.goal_tolerance = 0.05
        self.obstacle_tolerance = 0.4
        self.current_goal = 0
        self.pause_duration = 2.0
        self.pause_start_time = None
        self.obstacle_clearance = 0.55
        self.avoid_linear = 0.05
        self.avoid_turn = 0.8
        self.avoid_active = False
        self.avoid_direction = 1.0
        self.emergency_stop = False
        self._terminal_settings = None
        
        # Control gains
        self.k_linear = 0.5
        self.k_angular = 1.5
        self.max_linear = 0.1
        self.max_angular = 0.8

        self.enable_keyboard_stop()

        self.get_logger().info('GoToGoal listening to /odom and publishing command to /cmd_vel')
    
    def odom_callback(self, Odom):
        position = Odom.pose.pose.position
        q = Odom.pose.pose.orientation
        orientation = np.arctan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))
        if self.Init:
            self.Init_ang = orientation
            self.globalAng = self.Init_ang
            Mrot = np.matrix([[np.cos(self.Init_ang), np.sin(self.Init_ang)],
                           [-np.sin(self.Init_ang),  np.cos(self.Init_ang)]])
            self.Init_pos.x = Mrot[0,0]*position.x + Mrot[0,1]*position.y
            self.Init_pos.y = Mrot[1,0]*position.x + Mrot[1,1]*position.y
            self.Init_pos.z = position.z
            self.Init = False
            self.get_logger().info("initialization done!!")
        
        M_rot = np.matrix([[np.cos(self.Init_ang), np.sin(self.Init_ang)],
                           [-np.sin(self.Init_ang),  np.cos(self.Init_ang)]])
        self.globalPos.x = M_rot[0,0]*position.x + M_rot[0,1]*position.y - self.Init_pos.x
        self.globalPos.y = M_rot[1,0]*position.x + M_rot[1,1]*position.y - self.Init_pos.y
        self.globalAng = self.normalize_angle(orientation - self.Init_ang)
        # self.get_logger().info(f"Global position: x={self.globalPos.x:.3f}, y={self.globalPos.y:.3f}, angle={self.globalAng:.3f}")

    def object_callback(self, msg):
        if self.avoid_active:
            return

        obstacle_distance = math.hypot(msg.x, msg.y)
        if not (0.05 < obstacle_distance < 998.0):
            self.has_obstacle = False
            return

        self.obstaclePos = msg
        self.obstaclePosGlobal.x, self.obstaclePosGlobal.y = self.body_to_global(msg.x, msg.y)
        self.has_obstacle = True
        
    def normalize_angle(self, angle):
        while angle > np.pi:
            angle -= 2 * np.pi
        while angle < -np.pi:
            angle += 2 * np.pi
        return angle

    def control_loop(self):
        self.check_keyboard_input()

        if self.emergency_stop:
            self.stop_robot()
            return
        if self.Init:
            return
        

        if self.current_goal >= len(self.waypoints):
            self.get_logger().info("All goals reached!")
            self.stop_robot()
            return
        
        goal_x, goal_y = self.waypoints[self.current_goal]
        dx = goal_x - self.globalPos.x
        dy = goal_y - self.globalPos.y
        distance_error = math.sqrt(dx**2 + dy**2)
        angle_to_goal = math.atan2(dy, dx)
        angle_error = self.normalize_angle(angle_to_goal - self.globalAng )

        if distance_error < self.goal_tolerance:
            self.stop_robot()
            if self.pause_start_time is None:
                self.get_logger().info(f"Goal {self.current_goal + 1} reached!")
                self.pause_start_time = self.get_clock().now()

            elapsed = (self.get_clock().now() - self.pause_start_time).nanoseconds * 1e-9
            if elapsed >= self.pause_duration:
                self.current_goal += 1
                self.pause_start_time = None
            return

        if self.pause_start_time is not None:
            self.pause_start_time = None

        if self.should_avoid_obstacle():
            cmd_msg = self.build_avoidance_cmd()
            self.cmd_pub.publish(cmd_msg)
            self.get_logger().info(
                f'Avoiding obstacle: pos=({self.obstaclePos.x:.3f}, {self.obstaclePos.y:.3f}), '
                f'cmd_vel: linear={cmd_msg.linear.x:.3f} m/s, angular={cmd_msg.angular.z:.3f} rad/s'
            )
            return
        
        linear = self.k_linear * distance_error
        angular = self.k_angular * angle_error

        linear = max(-self.max_linear, min(self.max_linear, linear))
        angular = max(-self.max_angular, min(self.max_angular, angular))
        cmd_msg = Twist()
        cmd_msg.linear.x = linear
        cmd_msg.angular.z = angular
        self.cmd_pub.publish(cmd_msg)
        self.get_logger().info(f'Odometry: global position=({self.globalPos.x:.3f}, {self.globalPos.y:.3f}), global angle={self.globalAng:.3f} rad Angle_to_goal: ({angle_to_goal}), angle error: {angle_error:.3f} rad, cmd_vel: linear={linear:.3f} m/s, angular={angular:.3f} rad/s')

    def body_to_global(self, x_body, y_body):
        x_global = self.globalPos.x + x_body * math.cos(self.globalAng) - y_body * math.sin(self.globalAng)
        y_global = self.globalPos.y + x_body * math.sin(self.globalAng) + y_body * math.cos(self.globalAng)
        return x_global, y_global

    def global_to_body(self, x_global, y_global):
        dx = x_global - self.globalPos.x
        dy = y_global - self.globalPos.y
        x_body = dx * math.cos(self.globalAng) + dy * math.sin(self.globalAng)
        y_body = -dx * math.sin(self.globalAng) + dy * math.cos(self.globalAng)
        return x_body, y_body

    def obstacle_distance_global(self):
        return math.hypot(self.obstaclePosGlobal.x - self.globalPos.x, self.obstaclePosGlobal.y - self.globalPos.y)

    def has_valid_obstacle(self):
        if not self.has_obstacle:
            return False
        obstacle_distance = self.obstacle_distance_global()
        return 0.05 < obstacle_distance < 998.0

    def should_avoid_obstacle(self):
        if self.avoid_active:
            if self.should_exit_avoidance():
                self.avoid_active = False
                self.has_obstacle = False
                return False
            return True

        if not self.has_valid_obstacle():
            return False

        obstacle_distance = self.obstacle_distance_global()
        obstacle_x_body, obstacle_y_body = self.global_to_body(self.obstaclePosGlobal.x, self.obstaclePosGlobal.y)

        if obstacle_x_body > 0.0 and obstacle_distance < self.obstacle_tolerance:
            self.avoid_active = True
            if abs(obstacle_y_body) < 0.05:
                self.avoid_direction = 1.0
            else:
                self.avoid_direction = -math.copysign(1.0, obstacle_y_body)
            return True

        return False

    def should_exit_avoidance(self):
        obstacle_distance = self.obstacle_distance_global()
        obstacle_x_body, _ = self.global_to_body(self.obstaclePosGlobal.x, self.obstaclePosGlobal.y)

        if obstacle_x_body < -0.15 and obstacle_distance > self.obstacle_clearance:
            return True

        return False

    def build_avoidance_cmd(self):
        obstacle_distance = self.obstacle_distance_global()
        obstacle_x_body, obstacle_y_body = self.global_to_body(self.obstaclePosGlobal.x, self.obstaclePosGlobal.y)
        cmd_msg = Twist()

        if obstacle_distance < 0.25:
            cmd_msg.linear.x = 0.0
            cmd_msg.angular.z = self.avoid_direction * self.max_angular
            return cmd_msg

        if obstacle_x_body > 0.0 and abs(obstacle_y_body) < 0.25:
            cmd_msg.linear.x = 0.02
            cmd_msg.angular.z = self.avoid_direction * self.max_angular
            return cmd_msg

        if obstacle_x_body > -0.10:
            cmd_msg.linear.x = self.avoid_linear
            cmd_msg.angular.z = self.avoid_direction * self.avoid_turn
            return cmd_msg

        cmd_msg.linear.x = self.avoid_linear
        cmd_msg.angular.z = -self.avoid_direction * 0.3
        return cmd_msg

    def stop_robot(self):
        msg = Twist()
        self.cmd_pub.publish(msg)

    def enable_keyboard_stop(self):
        try:
            self._terminal_settings = termios.tcgetattr(sys.stdin)
            tty.setcbreak(sys.stdin.fileno())
            self.get_logger().info('Keyboard stop enabled: press SPACE to stop the robot.')
        except Exception as exc:
            self._terminal_settings = None
            self.get_logger().warn(f'Keyboard stop disabled: {exc}')

    def check_keyboard_input(self):
        if self._terminal_settings is None or self.emergency_stop:
            return

        try:
            readable, _, _ = select.select([sys.stdin], [], [], 0.0)
            if readable:
                key = sys.stdin.read(1)
                if key == ' ':
                    self.emergency_stop = True
                    self.stop_robot()
                    self.get_logger().warn('SPACE pressed: emergency stop activated.')
        except Exception as exc:
            self.get_logger().warn(f'Keyboard read failed: {exc}')

    def destroy_node(self):
        if self._terminal_settings is not None:
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self._terminal_settings)
        return super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = GoToGoal()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
