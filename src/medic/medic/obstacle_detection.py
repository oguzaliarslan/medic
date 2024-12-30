#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
from nav_msgs.msg import OccupancyGrid
from visualization_msgs.msg import Marker, MarkerArray
import numpy as np
import math

class ObstacleDetectionNode(Node):
    def __init__(self):
        super().__init__('obstacle_detection_node')
        
        self.scan_sub = self.create_subscription(
            Float32MultiArray,
            'processed_scan',
            self.processed_scan_callback,
            10
        )
        
        self.obstacle_markers_pub = self.create_publisher(
            MarkerArray,
            'detected_obstacles',
            10
        )
        
        self.costmap_pub = self.create_publisher(
            OccupancyGrid,
            'local_costmap',
            10
        )
        
        # Parameters
        self.obstacle_threshold = 0.5  # meters
        self.init_costmap()
        
        self.get_logger().info('Obstacle Detection Node Initialized')

    def init_costmap(self):
        """Initialize costmap"""
        self.costmap = OccupancyGrid()
        self.costmap.header.frame_id = 'base_link'
        self.costmap.info.resolution = 0.05
        self.costmap.info.width = 100  # 5m / 0.05
        self.costmap.info.height = 100
        self.costmap.info.origin.position.x = -2.5
        self.costmap.info.origin.position.y = -2.5
        self.costmap.data = [0] * (self.costmap.info.width * self.costmap.info.height)

    def processed_scan_callback(self, msg):
        """Handle processed scan data"""
        points = []
        for i in range(0, len(msg.data), 3):
            x, y, r = msg.data[i:i+3]
            if r < self.obstacle_threshold and not math.isinf(r):
                points.append((x, y))

        # Update visualizations and costmap
        self.update_costmap(points)
        self.publish_markers(points)

    def update_costmap(self, points):
        """Update costmap with detected obstacles"""
        # Reset costmap
        self.costmap.data = [0] * (self.costmap.info.width * self.costmap.info.height)
        
        for x, y in points:
            # Convert to grid coordinates
            grid_x = int((x - self.costmap.info.origin.position.x) / self.costmap.info.resolution)
            grid_y = int((y - self.costmap.info.origin.position.y) / self.costmap.info.resolution)
            
            if 0 <= grid_x < self.costmap.info.width and 0 <= grid_y < self.costmap.info.height:
                index = grid_y * self.costmap.info.width + grid_x
                self.costmap.data[index] = 100
                
                # Inflate obstacles
                self.inflate_obstacle(grid_x, grid_y)
        
        self.costmap.header.stamp = self.get_clock().now().to_msg()
        self.costmap_pub.publish(self.costmap)

    def inflate_obstacle(self, x, y, radius=3):
        """Inflate obstacles in costmap"""
        for dx in range(-radius, radius + 1):
            for dy in range(-radius, radius + 1):
                if dx*dx + dy*dy <= radius*radius:
                    new_x, new_y = x + dx, y + dy
                    if 0 <= new_x < self.costmap.info.width and 0 <= new_y < self.costmap.info.height:
                        index = new_y * self.costmap.info.width + new_x
                        self.costmap.data[index] = max(50, self.costmap.data[index])

    def publish_markers(self, points):
        """Publish visualization markers"""
        marker_array = MarkerArray()
        
        for i, (x, y) in enumerate(points):
            marker = Marker()
            marker.header.frame_id = "base_link"
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.id = i
            marker.type = Marker.CYLINDER
            marker.action = Marker.ADD
            
            marker.pose.position.x = x
            marker.pose.position.y = y
            marker.pose.position.z = 0.1
            
            marker.scale.x = 0.1
            marker.scale.y = 0.1
            marker.scale.z = 0.2
            
            marker.color.r = 1.0
            marker.color.a = 0.8
            
            marker_array.markers.append(marker)
        
        self.obstacle_markers_pub.publish(marker_array)

def main(args=None):
    rclpy.init(args=args)
    node = ObstacleDetectionNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()