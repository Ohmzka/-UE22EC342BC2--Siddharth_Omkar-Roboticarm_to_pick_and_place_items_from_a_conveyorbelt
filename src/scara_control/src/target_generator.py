#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose
import random

class TargetGenerator(Node):
    def __init__(self):
        super().__init__('target_generator')
        self.publisher_ = self.create_publisher(Pose, '/target_pose', 10)
        self.timer_ = self.create_timer(5.0, self.publish_target)

    def publish_target(self):
        msg = Pose()
        msg.position.x = random.uniform(0.2, 0.5)
        msg.position.y = random.uniform(-0.2, 0.2)
        msg.position.z = 0.1
        self.publisher_.publish(msg)
        self.get_logger().info(f'Published target pose: {msg.position.x:.2f}, {msg.position.y:.2f}, {msg.position.z:.2f}')

def main(args=None):
    rclpy.init(args=args)
    node = TargetGenerator()
    rclpy.spin(node)
    rclpy.shutdown()

