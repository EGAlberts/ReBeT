from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():

    config_file = os.path.join(
    get_package_share_directory('rebet'),
    'config',
    'rebet_config.yaml'
    )
    return LaunchDescription([
        Node(
            package='rebet',
            executable='SLAM_action_server.py',
            name='slam_action_server',
            parameters=[config_file,{}]
        ),
        Node(
            package='rebet',
            executable='bandit_action_server.py',
            name='bandit_action_server',
            parameters=[config_file,{}]
        ),
        Node(
            package='rebet',
            executable='ID_action_server.py',
            name='identify_action_server',
            parameters=[config_file,{}]
        ),
        Node(
            package='rebet',
            executable='reactive_action_server.py',
            name='reactive_action_server',
            parameters=[config_file,{}]
        ),
        Node(
            package='rebet',
            executable='random_action_server.py',
            name='random_action_server',
            parameters=[config_file,{}]
        ),
    ])