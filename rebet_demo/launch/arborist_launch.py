from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import IncludeLaunchDescription

def generate_launch_description():
    config_file = os.path.join(
    get_package_share_directory('rebet_demo'),
    'config',
    'rebet_config.yaml'
    )

    spawn_models_launch_file = os.path.join(
        get_package_share_directory('rebet_demo'),
        'launch',
        'spawn_models_launch.py'
    )

    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(spawn_models_launch_file)
        ),
        Node(
            package='rebet_demo',
            executable='arborist',
            name='arborist_node',
            parameters=[config_file],
            output='screen'
        ),
    ])