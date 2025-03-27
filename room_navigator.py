#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from std_msgs.msg import String
from .room_config import ROOM_COORDINATES
import math
from tf_transformations import quaternion_from_euler

class RoomNavigator(Node):
    def __init__(self):
        super().__init__('room_navigator')
        
        # Create action client for Nav2
        self.nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        
        # Subscribe to room commands
        self.command_sub = self.create_subscription(
            String,
            'room_command',
            self.command_callback,
            10)
        
        self.get_logger().info('Room Navigator started')

    def command_callback(self, msg):
        # Extract room name from command
        command = msg.data.lower()
        room = command.replace('go to ', '').strip()
        
        if room in ROOM_COORDINATES:
            self.get_logger().info(f'Navigating to {room}')
            self.navigate_to_room(room)
        else:
            self.get_logger().info(f'Unknown room: {room}')
            
    def navigate_to_room(self, room_name):
        # Create navigation goal
        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = 'map'
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        
        # Set position
        goal.pose.pose.position.x = ROOM_COORDINATES[room_name]['x']
        goal.pose.pose.position.y = ROOM_COORDINATES[room_name]['y']
        goal.pose.pose.position.z = 0.0
        
        # Convert euler angle to quaternion
        q = quaternion_from_euler(0, 0, ROOM_COORDINATES[room_name]['orientation'])
        goal.pose.pose.orientation.x = q[0]
        goal.pose.pose.orientation.y = q[1]
        goal.pose.pose.orientation.z = q[2]
        goal.pose.pose.orientation.w = q[3]
        
        # Send goal
        self.nav_client.wait_for_server()
        self.send_goal(goal)
        
    def send_goal(self, goal):
        self.get_logger().info('Sending goal')
        send_goal_future = self.nav_client.send_goal_async(goal)
        send_goal_future.add_done_callback(self.goal_response_callback)
        
    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('Goal rejected')
            return
            
        self.get_logger().info('Goal accepted')
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.get_result_callback)
        
    def get_result_callback(self, future):
        status = future.result().status
        if status == 4:  # succeeded
            self.get_logger().info('Navigation successful!')
        else:
            self.get_logger().error(f'Navigation failed with status: {status}')

def main(args=None):
    rclpy.init(args=args)
    navigator = RoomNavigator()
    rclpy.spin(navigator)
    navigator.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
