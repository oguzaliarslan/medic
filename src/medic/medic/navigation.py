#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import OccupancyGrid
from sensor_msgs.msg import LaserScan
import numpy as np
import math
import random

class ExplorationNode(Node):
    def __init__(self):
        super().__init__('navigation_node')
        
        self.cmd_vel_pub = self.create_publisher(
            Twist,
            'cmd_vel',
            10
        )
        
        self.scan_sub = self.create_subscription(
            LaserScan,
            'scan',
            self.scan_callback,
            10
        )
        
        self.linear_speed = 0.2
        self.angular_speed = 0.5
        self.min_distance = 0.5 
        self.scan_data = None
        
        self.state = 'FORWARD'  # States: FORWARD, ROTATING
        self.rotate_time = 0
        self.forward_time = 0
        
        # Create timer for movement updates
        self.create_timer(0.1, self.exploration_step)  # 10Hz
        
        self.get_logger().info('Exploration Node Initialized')

    def scan_callback(self, msg):
        """Handle laser scan data"""
        self.scan_data = msg

    def is_path_clear(self):
        """Check if the path ahead is clear"""
        if not self.scan_data:
            return False
            
        # Check the front sector (±30 degrees)
        front_indices = len(self.scan_data.ranges) // 6  # 60 degrees total
        center_index = len(self.scan_data.ranges) // 2
        front_ranges = self.scan_data.ranges[center_index - front_indices:
                                           center_index + front_indices]
        
        # Filter out invalid readings
        valid_ranges = [r for r in front_ranges 
                       if not math.isinf(r) and not math.isnan(r)]
        
        if not valid_ranges:
            return False
            
        return min(valid_ranges) > self.min_distance

    def find_clear_direction(self):
        """Find a clear direction to move"""
        if not self.scan_data:
            return random.uniform(-math.pi/2, math.pi/2)
            
        num_sectors = 8
        sector_size = len(self.scan_data.ranges) // num_sectors
        sector_distances = []
        
        for i in range(num_sectors):
            start_idx = i * sector_size
            end_idx = (i + 1) * sector_size
            sector_ranges = [r for r in self.scan_data.ranges[start_idx:end_idx]
                           if not math.isinf(r) and not math.isnan(r)]
            
            if sector_ranges:
                sector_distances.append(sum(sector_ranges) / len(sector_ranges))
            else:
                sector_distances.append(0.0)
        
        max_distance = max(sector_distances)
        clear_sectors = [i for i, d in enumerate(sector_distances) 
                        if d > max_distance * 0.8]
        
        if clear_sectors:
            # Choose a random clear sector
            chosen_sector = random.choice(clear_sectors)
            return (chosen_sector - num_sectors/2) * (2*math.pi/num_sectors)
        else:
            return random.uniform(-math.pi/2, math.pi/2)

    def exploration_step(self):
        """Execute one step of exploration"""
        if not self.scan_data:
            self.stop_robot()
            return
            
        cmd = Twist()
        
        if self.state == 'FORWARD':
            if self.is_path_clear():
                # Move forward
                cmd.linear.x = self.linear_speed
                cmd.angular.z = 0.0
                
                # Randomly decide to change direction
                if random.random() < 0.02:  # 2% chance per step
                    self.state = 'ROTATING'
                    self.rotate_time = random.uniform(0.5, 2.0)  # Random rotation duration
                    self.get_logger().info('Changing direction randomly')
            else:
                # Path blocked, start rotating
                self.state = 'ROTATING'
                self.rotate_time = random.uniform(1.0, 2.0)
                self.get_logger().info('Obstacle detected, rotating')
        
        elif self.state == 'ROTATING':
            # Rotate until we find a clear path
            cmd.linear.x = 0.0
            clear_direction = self.find_clear_direction()
            cmd.angular.z = self.angular_speed if clear_direction > 0 else -self.angular_speed
            
            self.rotate_time -= 0.1  # Decrease remaining rotation time
            if self.rotate_time <= 0 and self.is_path_clear():
                self.state = 'FORWARD'
                self.get_logger().info('Found clear path, moving forward')
        
        self.cmd_vel_pub.publish(cmd)

    def stop_robot(self):
        """Stop the robot"""
        cmd = Twist()
        cmd.linear.x = 0.0
        cmd.angular.z = 0.0
        self.cmd_vel_pub.publish(cmd)

def main(args=None):
    rclpy.init(args=args)
    node = ExplorationNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.stop_robot()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()