#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class AppTopicsMonitor(Node):
    def __init__(self):
        super().__init__('app_topics_monitor')
        
        # Create publishers to ensure topics exist
        self.addr_pub = self.create_publisher(String, '/app/address', 10)
        self.phone_pub = self.create_publisher(String, '/app/phone', 10)
        
        # Subscribe to echo messages
        self.addr_sub = self.create_subscription(
            String, '/app/address',
            lambda msg: self.get_logger().info(f'Address received: {msg.data}'),
            10)
        self.phone_sub = self.create_subscription(
            String, '/app/phone',
            lambda msg: self.get_logger().info(f'Phone received: {msg.data}'),
            10)
        
        self.get_logger().info('App topics monitor ready')

def main(args=None):
    rclpy.init(args=args)
    node = AppTopicsMonitor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
