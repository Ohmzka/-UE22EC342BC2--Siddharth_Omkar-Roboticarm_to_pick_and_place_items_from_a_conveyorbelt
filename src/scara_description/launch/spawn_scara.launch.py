from launch import LaunchDescription
from launch.actions import TimerAction
from launch_ros.actions import Node
from launch.substitutions import Command, PathJoinSubstitution, FindExecutable
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Paths
    pkg_scara_description = get_package_share_directory('scara_description')
    pkg_scara_control = get_package_share_directory('scara_control')

    robot_description_content = Command([
        FindExecutable(name='xacro'), ' ',
        PathJoinSubstitution([pkg_scara_description, 'urdf', 'scara1.urdf.xacro']),
        ' use_paused_simulation:=true'
    ])
    robot_description = {'robot_description': robot_description_content}

    # Controller configuration
    controller_config = PathJoinSubstitution(
        [pkg_scara_control, 'config', 'scara_controllers.yaml']
    )

    # Gazebo with paused physics
    gazebo_server = ExecuteProcess(
        cmd=['gzserver', '--verbose', '--pause',
             '-s', 'libgazebo_ros_init.so',
             '-s', 'libgazebo_ros_factory.so'],
        output='screen'
    )

    # Main launch sequence
    return LaunchDescription([
        # Start Gazebo paused
        gazebo_server,
        ExecuteProcess(cmd=['gzclient'], output='screen'),
        
        # Robot State Publisher
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            output='screen',
            parameters=[robot_description, {'use_sim_time': True}]
        ),
        
        # Spawn robot with precise positioning
        Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            arguments=[
                '-entity', 'scara',
                '-topic', 'robot_description',
                '-x', '0', '-y', '0', '-z', '0.01',  # Virtually on ground
                '-R', '0', '-P', '0', '-Y', '0' ,
                '-param', 'static:=true'         # Explicit orientation
            ],
            output='screen'
        ),
        
        # Unpause physics after spawn
        TimerAction(
            period=3.0,
            actions=[
                ExecuteProcess(
                    cmd=['gz', 'physics', '-u', '1'],  # Unpause
                    output='screen'
                )
            ]
        ),
        
        # Start controller manager
        TimerAction(
            period=5.0,
            actions=[
                Node(
                    package='controller_manager',
                    executable='ros2_control_node',
                    parameters=[robot_description, controller_config],
                    output='screen'
                )
            ]
        ),
        
        # Spawn controllers with delays
        TimerAction(
            period=7.0,
            actions=[
                Node(
                    package='controller_manager',
                    executable='spawner',
                    arguments=['joint_state_broadcaster'],
                    output='screen'
                )
            ]
        ),
        TimerAction(
            period=9.0,
            actions=[
                Node(
                    package='controller_manager',
                    executable='spawner',
                    arguments=['joint1_position_controller'],
                    output='screen'
                ),
                Node(
                    package='controller_manager',
                    executable='spawner',
                    arguments=['joint2_position_controller'],
                    output='screen'
                ),
                Node(
                    package='controller_manager',
                    executable='spawner',
                    arguments=['joint3_position_controller'],
                    output='screen'
                )
            ]
        )
    ])
