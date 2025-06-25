#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # Declare launch arguments
    port_arg = DeclareLaunchArgument(
        'port',
        default_value='9090',
        description='Port for ros2bridge websocket server'
    )
    
    address_arg = DeclareLaunchArgument(
        'address',
        default_value='0.0.0.0',
        description='Address for ros2bridge websocket server'
    )

    # Create ros2bridge node
    ros2bridge_node = Node(
        package='ros2bridge',
        executable='ros2bridge',
        name='ros2bridge',
        parameters=[{
            'port': LaunchConfiguration('port'),
            'address': LaunchConfiguration('address'),
        }],
        output='screen'
    )

    return LaunchDescription([
        port_arg,
        address_arg,
        ros2bridge_node,
    ]) 