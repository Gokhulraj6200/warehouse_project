import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, ExecuteProcess
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def launch_setup(command, *args, **kwargs):
    map_file = command.launch_configurations['map_file']

    # Default values for map and RViz config directories
    map_path = os.path.join(get_package_share_directory('map_server'), 'config', map_file)

    if map_file == 'warehouse_map_real.yaml':
        rviz_config_dir = os.path.join(get_package_share_directory('localization_server'), 'rviz', 'real_localization.rviz')
        localization_yaml = os.path.join(get_package_share_directory('localization_server'), 'config', 'real_amcl_config.yaml')
        service_server = 'approach_service_server_real'
        execute_process = []
        use_sim_time = False
    else:
        rviz_config_dir = os.path.join(get_package_share_directory('localization_server'), 'rviz', 'sim_localization.rviz')
        localization_yaml = os.path.join(get_package_share_directory('localization_server'), 'config', 'amcl_config.yaml')
        service_server = 'approach_service_server_sim'
        execute_process = []
        use_sim_time = True

    return [
        Node(
            package='nav2_map_server',
            executable='map_server',
            name='map_server',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time}, 
                        {'yaml_filename': map_path}]),

        Node(
            package='nav2_amcl',
            executable='amcl',
            name='amcl',
            output='screen',
            parameters=[localization_yaml]),

        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_mapper',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time},
                        {'autostart': True},
                        {'node_names': ['map_server', 'amcl']}]),
        
        Node(
            package='attach_shelf',
            executable= service_server,
            name= service_server,
            output='screen'),
        
        *execute_process

    ]

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'map_file',
            default_value='warehouse_map_real.yaml',
            description='Type of map to use.'),
        OpaqueFunction(function=launch_setup)
    ])