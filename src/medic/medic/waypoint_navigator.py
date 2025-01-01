#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped
from rclpy.duration import Duration

class WaypointNavigator(Node):
    def __init__(self):
        super().__init__('waypoint_navigator')
        
        # Create Nav2 action client
        self.nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        
        # Declare and get parameters
        self.declare_parameter('waypoint_x', [1.0, 3.0, 2.0])
        self.declare_parameter('waypoint_y', [1.0, 1.0, 3.0])
        self.declare_parameter('waypoint_yaw', [0.0, 0.0, 0.0])
        self.declare_parameter('waypoint_names', ['Room1', 'Room2', 'Room3'])
        
        self.waypoint_x = self.get_parameter('waypoint_x').value
        self.waypoint_y = self.get_parameter('waypoint_y').value
        self.waypoint_yaw = self.get_parameter('waypoint_yaw').value
        self.waypoint_names = self.get_parameter('waypoint_names').value
        
        self.current_waypoint = 0
        self.num_waypoints = len(self.waypoint_x)
        
        # Wait for Nav2 action server
        self.get_logger().info('Waiting for Nav2 action server...')
        self.nav_client.wait_for_server()
        self.get_logger().info('Nav2 action server connected!')
        
        # Start navigation
        self.navigate_to_next_waypoint()
    
    def navigate_to_next_waypoint(self):
        if self.num_waypoints == 0:
            self.get_logger().warn('No waypoints defined!')
            return
            
        # Get current waypoint
        x = self.waypoint_x[self.current_waypoint]
        y = self.waypoint_y[self.current_waypoint]
        yaw = self.waypoint_yaw[self.current_waypoint]
        name = self.waypoint_names[self.current_waypoint]
        
        self.get_logger().info(f'Navigating to {name} at ({x}, {y})...')
        
        # Create goal pose
        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = 'map'
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        
        goal.pose.pose.position.x = float(x)
        goal.pose.pose.position.y = float(y)
        goal.pose.pose.orientation.w = 1.0
        
        # Send goal
        self.nav_client.send_goal_async(
            goal,
            feedback_callback=self.feedback_callback
        ).add_done_callback(self.goal_response_callback)
    
    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('Goal rejected!')
            return
            
        self.get_logger().info('Goal accepted!')
        goal_handle.get_result_async().add_done_callback(self.get_result_callback)
    
    def get_result_callback(self, future):
        result = future.result().result
        self.get_logger().info('Goal reached!')
        
        # Move to next waypoint
        self.current_waypoint = (self.current_waypoint + 1) % self.num_waypoints
        self.navigate_to_next_waypoint()
    
    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        # You can add custom feedback handling here if needed

def main(args=None):
    rclpy.init(args=args)
    node = WaypointNavigator()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()