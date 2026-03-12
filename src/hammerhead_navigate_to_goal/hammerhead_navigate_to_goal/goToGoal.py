import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point, Twist
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Quaternion
from rclpy.qos import qos_profile_sensor_data
import math
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

        # self.state_pub = self.create_publisher(Point, '/robot_state', 10)
        self.odom_sub = self.create_subscription(
            Odometry, '/odom', self.odom_callback, 10
        )
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.timer = self.create_timer(0.1, self.control_loop)
        self.waypoints = [[0.0, 1.4], [1.5, 1.4], [0.0, 1.4]]
        self.goal_tolerance = 0.1
        self.current_goal = 0
        self.pause_duration = 1.0
        self.pause_start_time = None
        
        # Control gains
        self.k_linear = 0.5
        self.k_angular = 3.0
        self.max_linear = 0.15
        self.max_angular = 1.2
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
        self.globalAng = orientation - self.Init_ang

        # self.get_logger().info(f'Odometry: global position=({self.globalPos.x:.3f}, {self.globalPos.y:.3f}), global angle={self.globalAng:.3f} rad')
    
    def normalize_angle(self, angle):
        while angle > np.pi:
            angle -= 2 * np.pi
        while angle < -np.pi:
            angle += 2 * np.pi
        return angle

    def control_loop(self):
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
        self.get_logger().info(f'Angle error to goal: {angle_error:.3f} rad')

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
        
        linear = self.k_linear * distance_error
        angular = self.k_angular * angle_error

        linear = max(-self.max_linear, min(self.max_linear, linear))
        angular = max(-self.max_angular, min(self.max_angular, angular))
        cmd_msg = Twist()
        cmd_msg.linear.x = linear
        cmd_msg.angular.z = angular
        self.cmd_pub.publish(cmd_msg)
        # self.get_logger().info(f'Published cmd_vel: linear={linear:.3f} m/s, angular={angular:.3f} rad/s, distance_error={distance_error:.3f} m, angle_error={angle_error:.3f} rad')
    
    def stop_robot(self):
        msg = Twist()
        self.cmd_pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = GoToGoal()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
