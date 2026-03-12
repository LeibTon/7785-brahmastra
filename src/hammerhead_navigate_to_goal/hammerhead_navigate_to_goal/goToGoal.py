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

        # self.state_pub = self.create_publisher(Point, '/robot_state', 10)
        self.odom_sub = self.create_subscription(
            Odometry, '/odom', self.odom_callback, 10
        )
        self.get_logger().info('GoToGoal listening to /odom and publishing /position and angle of the object.')
    
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
        self.globalPos.z = orientation - self.Init_ang

        self.get_logger().info(f'Odometry: global position=({self.globalPos.x:.3f}, {self.globalPos.y:.3f}), global angle={self.globalPos.z:.3f} rad')

def main(args=None):
    rclpy.init(args=args)
    node = GoToGoal()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
