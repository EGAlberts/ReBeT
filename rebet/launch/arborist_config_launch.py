from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
def generate_launch_description():
    config_file = os.path.join(
    get_package_share_directory('rebet'),
    'config',
    'rebet_config.yaml'
    )

    return LaunchDescription([
        Node(
            package='rebet',
            executable='arborist',
            name='arborist_node',
            parameters=[config_file,{}]
        ),
    ])