from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, LogInfo
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Get the path to the conveyor.xacro
    pkg_path = get_package_share_directory('conveyor_description')
    xacro_file = os.path.join(pkg_path, 'urdf', 'conveyor.urdf.xacro')

    # Create the launch description
    return LaunchDescription([
        # Spawn conveyor in Gazebo
        Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            arguments=[
                '-entity', 'conveyor_belt',
                '-file', xacro_file,
                '-x', '-2', '-y', '-8', '-z', '0.1'
            ],
            output='screen'
        ),
        LogInfo(
            condition=None,
            msg="Gazebo Conveyor Belt and Spawn launched!"
        )
    ])

