#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose
from sensor_msgs.msg import JointState
import math

class DummyIKNode(Node):
    def __init__(self):
        super().__init__('dummy_ik_node')
        self.subscription = self.create_subscription(
            Pose,
            'target_pose',
            self.target_callback,
            10
        )
        self.joint_publisher = self.create_publisher(JointState, 'joint_states', 10)

    def target_callback(self, msg):
        x = msg.position.x
        y = msg.position.y

        # Fake IK for 2-link planar SCARA
        l1 = 0.3
        l2 = 0.3
        d = (x**2 + y**2 - l1**2 - l2**2) / (2 * l1 * l2)
        d = max(min(d, 1.0), -1.0)  # Clamp to avoid domain error

        try:
            theta2 = math.acos(d)
            theta1 = math.atan2(y, x) - math.atan2(l2 * math.sin(theta2), l1 + l2 * math.cos(theta2))
        except ValueError:
            self.get_logger().warn('Target out of reach')
            return

        joint_state = JointState()
        joint_state.name = ['joint1', 'joint2']
        joint_state.position = [theta1, theta2]
        joint_state.header.stamp = self.get_clock().now().to_msg()

        self.joint_publisher.publish(joint_state)
        self.get_logger().info(f'Published Joint Angles: θ1={theta1:.2f}, θ2={theta2:.2f}')

def main(args=None):
    rclpy.init(args=args)
    node = DummyIKNode()
    rclpy.spin(node)
    rclpy.shutdown()

