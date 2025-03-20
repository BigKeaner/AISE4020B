#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import time

class SimpleDriver(Node):
    def __init__(self):
        super().__init__('simple_driver')
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)

    def move_forward(self, speed=0.1, duration=3.0):
        twist = Twist()
        twist.linear.x = speed
        twist.angular.z = 0.0

        # Publish the forward command repeatedly for 'duration' seconds
        start_time = time.time()
        while (time.time() - start_time) < duration:
            self.cmd_vel_pub.publish(twist)
            time.sleep(0.1)

        # Stop
        twist.linear.x = 0.0
        twist.angular.z = 0.0
        self.cmd_vel_pub.publish(twist)
        self.get_logger().info('Stop command sent')

def main(args=None):
    rclpy.init(args=args)
    node = SimpleDriver()

    node.get_logger().info('Moving forward for 3 seconds...')
    node.move_forward(speed=0.1, duration=3.0)

    rclpy.shutdown()

if __name__ == '__main__':
    main()
