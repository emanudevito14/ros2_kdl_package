from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    iiwa_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('iiwa_bringup'), 'launch', 'iiwa.launch.py')
        ),
        launch_arguments={
            'robot_controller': 'velocity_controller',
            'command_interface': 'velocity'
        }.items()
    )

    kdl_server_node = Node(
        package='ros2_kdl_package',
        executable='kdl_action_server',
        name='kdl_server',
        output='screen',
        parameters=[{'robot_description': '/robot_description'}]
    )

    kdl_client_node = Node(
        package='ros2_kdl_package',
        executable='kdl_action_client',
        name='kdl_client',
        output='screen'
    )

    return LaunchDescription([
        iiwa_launch,
        kdl_server_node,
        kdl_client_node
    ])

