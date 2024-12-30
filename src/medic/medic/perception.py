#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float32MultiArray, MultiArrayDimension
import numpy as np
import math

class PerceptionNode(Node):
    def __init__(self):
        super().__init__('perception_node')
        
        # Subscriber for raw sensor data
        self.scan_subscription = self.create_subscription(
            LaserScan,
            'scan',
            self.scan_callback,
            10
        )
        
        # Publisher for processed sensor data
        self.processed_scan_pub = self.create_publisher(
            Float32MultiArray,
            'processed_scan',
            10
        )
        
        # Parameters for processing
        self.min_range = 0.2  # Minimum valid range
        self.max_range = 3.5  # Maximum valid range
        
        self.get_logger().info('Perception Node Initialized')

    def process_scan(self, scan_msg):
        """Process raw scan data into usable form"""
        processed_data = []
        angle = scan_msg.angle_min
        
        for r in scan_msg.ranges:
            if self.min_range <= r <= self.max_range:
                # Convert to Cartesian coordinates
                x = r * math.cos(angle)
                y = r * math.sin(angle)
                processed_data.extend([x, y, r])  # Store x, y, and range
            else:
                processed_data.extend([float('inf'), float('inf'), float('inf')])
            angle += scan_msg.angle_increment

        return processed_data

    def create_processed_message(self, processed_data):
        """Create Float32MultiArray message with processed data"""
        processed_msg = Float32MultiArray()
        processed_msg.data = processed_data
        
        # Set up array dimension (for proper data interpretation)
        processed_msg.layout.dim = [MultiArrayDimension()]
        processed_msg.layout.dim[0].label = "points"
        processed_msg.layout.dim[0].size = len(processed_data) // 3
        processed_msg.layout.dim[0].stride = 3
        
        return processed_msg

    def analyze_sectors(self, processed_data):
        """Analyze different sectors of the scan"""
        # Divide processed data into sectors
        sectors = {}
        points_per_sector = len(processed_data) // 9  # Divide into 8 sectors
        
        for i in range(8):
            start_idx = i * points_per_sector * 3
            end_idx = (i + 1) * points_per_sector * 3
            sector_data = processed_data[start_idx:end_idx]
            
            # Calculate minimum distance in this sector
            ranges = [sector_data[i+2] for i in range(0, len(sector_data), 3)
                     if not math.isinf(sector_data[i+2])]
            
            if ranges:
                sectors[i] = min(ranges)
            else:
                sectors[i] = float('inf')
        
        return sectors

    def scan_callback(self, msg: LaserScan):
        """Handle incoming laser scan data"""
        # Process the raw scan data
        processed_data = self.process_scan(msg)
        
        # Analyze sectors
        sectors = self.analyze_sectors(processed_data)
        
        # Create and publish processed message
        processed_msg = self.create_processed_message(processed_data)
        self.processed_scan_pub.publish(processed_msg)
        
        # Log sector analysis
        closest_sector = min(sectors.items(), key=lambda x: x[1])
        self.get_logger().debug(
            f'Closest obstacle in sector {closest_sector[0]} '
            f'at distance {closest_sector[1]:.2f}m'
        )

def main(args=None):
    rclpy.init(args=args)
    perception_node = PerceptionNode()
    
    try:
        rclpy.spin(perception_node)
    except KeyboardInterrupt:
        pass
    finally:
        perception_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()