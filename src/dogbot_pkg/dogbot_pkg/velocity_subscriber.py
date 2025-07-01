#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

class VelocitySubscriber(Node):
    def __init__(self):
        super().__init__('velocity_subscriber')
        self.subscription = self.create_subscription(
            Twist,
            'cmd_vel',
            self.listener_callback,
            10)
        
    def listener_callback(self, msg):
        self.get_logger().info(f'Received: Linear X: {msg.linear.x}, Angular Z: {msg.angular.z}')

def main(args=None):
    rclpy.init(args=args)
    subscriber = VelocitySubscriber()
    rclpy.spin(subscriber)
    subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()