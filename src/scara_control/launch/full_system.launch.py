from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os
from launch.substitutions import Command, FindExecutable
from launch_ros.parameter_descriptions import ParameterValue
import xacro

def generate_launch_description():
    # Package paths
    scara_description_pkg = get_package_share_directory('scara_description')
    conveyor_description_pkg = get_package_share_directory('conveyor_description')
    scara_gazebo_pkg = get_package_share_directory('scara_gazebo')
    scara_control_pkg = get_package_share_directory('scara_control')

    # Paths to URDFs/XACROs
    conveyor_xacro_path = os.path.join(conveyor_description_pkg, 'urdf', 'conveyor.urdf.xacro')
    scara_xacro_path = os.path.join(scara_description_pkg, 'urdf', 'scara1.urdf.xacro')

    # Controller config
    scara_controller_config = os.path.join(scara_control_pkg, 'config', 'scara_controllers.yaml')

    # Process XACRO for Conveyor
    conveyor_description_content = xacro.process_file(conveyor_xacro_path).toxml()
    conveyor_description = {
        "robot_description": conveyor_description_content,
        "use_sim_time": True
    }

    # Include Gazebo world
    gazebo_world = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(scara_gazebo_pkg, 'launch', 'scara_world.launch.py')
        )
    )

    # Robot State Publisher for SCARA
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{
            'robot_description': ParameterValue(
                Command([
                    FindExecutable(name='xacro'), ' ', scara_xacro_path
                ]),
                value_type=str
            ),
            'use_sim_time': True
        }],
        output='screen'
    )

    # Robot State Publisher for Conveyor
    conveyor_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='conveyor_state_publisher',
        output='screen',
        parameters=[conveyor_description]
    )

    # SCARA ros2_control_node
    scara_control_node = Node(
        package='controller_manager',
        executable='ros2_control_node',
        namespace='scara',
        parameters=[
            {
                'robot_description': ParameterValue(
                    Command([
                        FindExecutable(name='xacro'), ' ', scara_xacro_path
                    ]),
                    value_type=str
                ),
                'use_sim_time': True
            },
            scara_controller_config
        ],
        output='screen'
    )

    # Conveyor ros2_control_node
    conveyor_control_node = Node(
        package='controller_manager',
        executable='ros2_control_node',
        namespace='conveyor',
        parameters=[conveyor_description],
        output='screen'
    )

    # Spawn SCARA robot
    spawn_scara = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-entity', 'scara',
            '-topic', 'robot_description',
            '-x', '0', '-y', '0', '-z', '0.01'
        ],
        output='screen'
    )

    # Spawn Conveyor robot
    spawn_conveyor = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-entity', 'conveyor',
            '-topic', 'conveyor_description',
            '-x', '1.5', '-y', '0', '-z', '0.01'
        ],
        output='screen'
    )

    # SCARA controllers
    spawn_scara_controllers = [
        Node(
            package='controller_manager',
            executable='spawner',
            arguments=[
                'joint_state_broadcaster',
                '--controller-manager', '/scara/controller_manager'
            ],
            output='screen'
        ),
        Node(
            package='controller_manager',
            executable='spawner',
            arguments=[
                'joint1_position_controller',
                '--controller-manager', '/scara/controller_manager'
            ],
            output='screen'
        ),
        Node(
            package='controller_manager',
            executable='spawner',
            arguments=[
                'joint2_position_controller',
                '--controller-manager', '/scara/controller_manager'
            ],
            output='screen'
        ),
        Node(
            package='controller_manager',
            executable='spawner',
            arguments=[
                'joint3_position_controller',
                '--controller-manager', '/scara/controller_manager'
            ],
            output='screen'
        )
    ]

    # Conveyor controller
    spawn_conveyor_controller = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'conveyor_controller',
            '--controller-manager', '/conveyor/controller_manager'
        ],
        output='screen'
    )

    # Target generator node
    target_generator = Node(
        package='scara_control',
        executable='target_generator.py',
        name='target_generator',
        output='screen'
    )

    return LaunchDescription([
        gazebo_world,

        robot_state_publisher,
        conveyor_state_publisher_node,

        TimerAction(period=1.0, actions=[scara_control_node]),
        TimerAction(period=2.0, actions=[conveyor_control_node]),

        TimerAction(period=4.0, actions=[spawn_scara]),
        TimerAction(period=5.0, actions=[spawn_conveyor]),

        TimerAction(period=7.0, actions=spawn_scara_controllers),
        TimerAction(period=9.0, actions=[spawn_conveyor_controller]),

        TimerAction(period=11.0, actions=[target_generator]),
    ])

