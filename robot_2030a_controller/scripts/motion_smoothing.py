#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray  
from std_msgs.msg import Int32MultiArray

class ArmActuateSubscriber(Node):
    def __init__(self):
        super().__init__('motion_smoothing')

        # Initialize buffers and counters
        self.raw_joint_1 = []
        self.raw_joint_2 = []
        self.raw_joint_3 = []
        self.raw_joint_4 = []
        self.raw_joint_5 = []
        self.raw_joint_6 = []
        self.buffer = [0, 0, 0, 0, 0, 0]
        self.counter = 2

        # Create publisher and subscriber
        self.control_pub = self.create_publisher(Int32MultiArray, 'smooth_motion_control', 10)
        self.create_subscription(Int32MultiArray, 'robot_cmd_pub_', self.callback, 10)

        # Logging initialization
        self.get_logger().info('Node initialized and ready.')

    def check_last_value_repeated(self, lst):
        if len(lst) == 0:
            return False

        last_value = lst[-1]
        count = lst.count(last_value)

        return count >= self.counter

    def callback(self, data):
        # Callback function to handle incoming messages
        raw_data = data.data

        self.raw_joint_1.append(raw_data[0])
        self.raw_joint_2.append(raw_data[1])
        self.raw_joint_3.append(raw_data[2])
        self.raw_joint_4.append(raw_data[3])
        self.raw_joint_5.append(raw_data[4])
        self.raw_joint_6.append(raw_data[5])

        self.process_joint(self.raw_joint_1, 0)
        self.process_joint(self.raw_joint_2, 1)
        self.process_joint(self.raw_joint_3, 2)
        self.process_joint(self.raw_joint_4, 3)
        self.process_joint(self.raw_joint_5, 4)
        self.process_joint(self.raw_joint_6, 5)

        self.publish_control_data()

    def process_joint(self, raw_joint, index):
        if raw_joint[-1] >= 0:
            result = self.check_last_value_repeated(raw_joint)
            if result:
                max_value = max(raw_joint)
                min_value = min(raw_joint)
                if max_value not in self.buffer:
                    self.buffer[index] = max_value
                    raw_joint.clear()
                else:
                    self.buffer[index] = min_value
                    raw_joint.clear()
        elif raw_joint[-1] < 0:
            result = self.check_last_value_repeated(raw_joint)
            if result:
                max_value = max(raw_joint)
                min_value = min(raw_joint)
                if min_value not in self.buffer:
                    self.buffer[index] = min_value
                    raw_joint.clear()
                else:
                    self.buffer[index] = max_value
                    raw_joint.clear()

    def publish_control_data(self):
        msg = Int32MultiArray(data=self.buffer)
        self.control_pub.publish(msg)
        # self.get_logger().info(f"Published joint control data: {self.buffer}")

def main(args=None):
    rclpy.init(args=args)
    node = ArmActuateSubscriber()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down node.')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
