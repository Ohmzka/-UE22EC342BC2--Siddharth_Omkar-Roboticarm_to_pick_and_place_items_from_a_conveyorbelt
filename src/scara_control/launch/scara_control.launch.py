from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='scara_control',
            executable='target_generator.py',
            name='target_generator',
            output='screen'
        ),
        Node(
            package='scara_control',
            executable='coordination_node',
            name='coordination_node',
            output='screen'
        ),
        Node(
            package='scara_control',
            executable='gripper_control.py',
            name='gripper_control',
            output='screen'
        ),
        Node(
            package='scara_control',
            executable='conveyor_control.py',
            name='conveyor_control',
            output='screen'
        )
    ])

