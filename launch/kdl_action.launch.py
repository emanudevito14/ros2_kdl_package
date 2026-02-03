import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    # Definiamo un argomento per il lancio, così puoi cambiarlo da terminale se serve
    # Esempio: ros2 launch ros2_kdl_package kdl_action.launch.py controller:=velocity_ctrl
    ctrl_arg = DeclareLaunchArgument(
        'controller',
        default_value='vision_ctrl',
        description='Tipo di controllore: velocity_ctrl, velocity_ctrl_null o vision_ctrl'
    )

    # Nodo Server
    server_node = Node(
        package='ros2_kdl_package',
        executable='kdl_action_server_node',  
        name='kdl_action_server_node',
        output='screen',
        parameters=[{
            'ctrl': LaunchConfiguration('controller'),
            'cmd_interface': 'velocity', 
            'Kp': 5.0
        }]
    )

    # Nodo Client
    client_node = Node(
        package='ros2_kdl_package',
        executable='kdl_action_client_node',  
        name='kdl_action_client_node',
        output='screen',
        parameters=[{
            'ctrl': LaunchConfiguration('controller')
        }]
    )

    return LaunchDescription([
        ctrl_arg,
        server_node,
        client_node
    ])
