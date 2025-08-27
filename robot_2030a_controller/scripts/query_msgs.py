#!/usr/bin/env python3

import rclpy
from std_msgs.msg import Float32MultiArray
from rclpy.node import Node

class QueryPublisher(Node):
    def __init__(self):
        super().__init__('query_publisher')
        # Create a publisher to the 'query_response' topic
        self.publisher_ = self.create_publisher(Float32MultiArray, 'query_response', 10)
        # Timer to publish messages at a fixed rate (e.g., every 1 second)
        self.timer = self.create_timer(1.0, self.timer_callback)

    def timer_callback(self):
        msg = Float32MultiArray()
        # Populate the message with sample data (6 float values)
        msg.data = [0.33, 0.0, 0.0, 0.0, 0.0, 0.0]
        # Log the message being published
        self.get_logger().info(f'Publishing: {msg.data}')
        # Publish the message
        self.publisher_.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    query_publisher = QueryPublisher()
    rclpy.spin(query_publisher)

    # Destroy the node once spinning is complete (clean-up)
    query_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
