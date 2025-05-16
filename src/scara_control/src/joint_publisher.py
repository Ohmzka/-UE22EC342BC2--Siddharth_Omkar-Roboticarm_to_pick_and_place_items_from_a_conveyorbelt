#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Header
from std_msgs.msg import Float64MultiArray

class JointPublisher(Node):
    def __init__(self):
        super().__init__('joint_publisher')
        self.joint_pub = self.create_publisher(JointState, '/joint_states', 10)
        self.subscription = self.create_subscription(
            Float64MultiArray,
            'scara_joint_angles',
            self.joint_callback,
            10
        )
        self.current_angles = [0.0, 0.0, 0.0, 0.0]
        self.timer = self.create_timer(0.05, self.publish_joints)

    def joint_callback(self, msg):
        if len(msg.data) == 4:
            self.current_angles = msg.data

    def publish_joints(self):
        js = JointState()
        js.header = Header()
        js.header.stamp = self.get_clock().now().to_msg()
        js.name = ['joint1', 'joint2', 'joint3', 'joint4']
        js.position = self.current_angles
        self.joint_pub.publish(js)

def main(args=None):
    rclpy.init(args=args)
    node = JointPublisher()
    rclpy.spin(node)
    rclpy.shutdown()

