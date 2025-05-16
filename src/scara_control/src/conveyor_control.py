#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64

class ConveyorControl(Node):
    def __init__(self):
        super().__init__('conveyor_control')
        self.publisher_ = self.create_publisher(Float64, '/conveyor_belt/command', 10)
        self.timer = self.create_timer(1.0, self.publish_velocity)

    def publish_velocity(self):
        msg = Float64()
        msg.data = 0.1  # Set conveyor velocity
        self.publisher_.publish(msg)
        self.get_logger().info(f'Sent velocity command: {msg.data}')

def main(args=None):
    rclpy.init(args=args)
    node = ConveyorControl()
    rclpy.spin(node)
    rclpy.shutdown()

