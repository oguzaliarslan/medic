#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from math import pi

class MedicRobot(Node):
    def __init__(self):
        super().__init__('medic_robot')
        
        # Create publisher for robot movement
        self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 10)
        
        # Create subscribers
        self.odom_subscription = self.create_subscription(
            Odometry,
            'odom',
            self.odom_callback,
            10)
        
        self.scan_subscription = self.create_subscription(
            LaserScan,
            'scan',
            self.scan_callback,
            10)
            
        # Initialize robot state
        self.current_pose = None
        self.current_scan = None
        
        self.get_logger().info('MEDIC Robot Controller Initialized')

    def odom_callback(self, msg):
        """Handle odometry data."""
        self.current_pose = msg.pose.pose
        
    def scan_callback(self, msg):
        """Handle laser scan data."""
        self.current_scan = msg

    def move(self, linear_velocity=0.0, angular_velocity=0.0):
        """Move the robot with given velocities."""
        msg = Twist()
        msg.linear.x = linear_velocity
        msg.angular.z = angular_velocity
        self.publisher_.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    medic_robot = MedicRobot()
    
    try:
        rclpy.spin(medic_robot)
    except KeyboardInterrupt:
        pass
    finally:
        # Stop the robot before shutting down
        medic_robot.move(0.0, 0.0)
        medic_robot.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()