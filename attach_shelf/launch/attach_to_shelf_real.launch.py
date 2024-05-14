import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument

def generate_launch_description():

    return LaunchDescription([

        Node(
            package='attach_shelf',
            executable='approach_service_server',
            name='approach_service_server'
        )
    ])