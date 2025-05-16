#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64

class GripperControl(Node):
    def __init__(self):
        super().__init__('gripper_control')
        self.gripper_pub = self.create_publisher(Float64, '/scara/gripper_controller/command', 10)
        self.timer = self.create_timer(5.0, self.toggle_grip)
        self.open = True

    def toggle_grip(self):
        msg = Float64()
        msg.data = 0.0 if self.open else 1.0
        self.gripper_pub.publish(msg)
        self.open = not self.open
        self.get_logger().info(f"{'Opened' if self.open else 'Closed'} gripper")

def main(args=None):
    rclpy.init(args=args)
    node = GripperControl()
    rclpy.spin(node)
    rclpy.shutdown()

