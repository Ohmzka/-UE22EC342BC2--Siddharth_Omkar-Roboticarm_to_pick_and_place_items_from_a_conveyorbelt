from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Package paths
    scara_description_pkg = get_package_share_directory('scara_description')
    conveyor_description_pkg = get_package_share_directory('conveyor_description')
    scara_gazebo_pkg = get_package_share_directory('scara_gazebo')
    scara_control_pkg = get_package_share_directory('scara_control')

    # Path to controller config file
    controller_config = os.path.join(scara_control_pkg, 'config', 'scara_controllers.yaml')

    # Launch Gazebo world
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(scara_gazebo_pkg, 'launch', 'scara_world.launch.py')
        )
    )

    # ros2_control controller manager node (must be launched for YAML to load)
    ros2_control_node = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[controller_config],
        output='screen'
    )

    # Spawn SCARA controllers
    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster'],
        output='screen'
    )

    joint_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['scara_position_controller'],
        output='screen'
    )
    
    
    # Spawn SCARA robot
    spawn_scara = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(scara_description_pkg, 'launch', 'spawn_scara.launch.py')
        )
    )

    # Spawn conveyor
    spawn_conveyor = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(conveyor_description_pkg, 'launch', 'conveyor_gz.launch.py')
        )
    )


    # Delay all robot control nodes until everything is spawned
    control_nodes = TimerAction(
        period=15.0,
        actions=[
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
            ),
            Node(
                package='scara_control',
                executable='spawn_objects.py',
                name='spawn_objects',
                output='screen'
            ),
            Node(
                package='scara_control',
                executable='ik_solver.py',
                name='ik_solver',
                output='screen'
            ),
        ]
    )

    return LaunchDescription([
        gazebo_launch,
        TimerAction(period=2.0, actions=[spawn_scara]),
        TimerAction(period=3.0, actions=[spawn_conveyor]),
        TimerAction(period=5.0, actions=[ros2_control_node]),
        TimerAction(period=7.0, actions=[joint_state_broadcaster_spawner]),
        TimerAction(period=9.0, actions=[joint_controller_spawner]),
        control_nodes
    ])
