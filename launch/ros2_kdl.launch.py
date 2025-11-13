from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    pkg_share = get_package_share_directory('ros2_kdl_package')

    params_file = os.path.join(pkg_share, 'config', 'kdl_params.yaml')

    return LaunchDescription([
        Node(
            package='ros2_kdl_package',
            executable='ros2_kdl_node',
            name='ros2_kdl_node',
            output='screen',
            parameters=[
                params_file,
                {"ctrl": "velocity_ctrl"}  # ➜ default: standard velocity control
            ]
        )
    ])

