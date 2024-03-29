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

    hyperparam_arg = DeclareLaunchArgument(
    'hyperparameters',
    description='Hyperparameters to the bandit'
    )
    hypparams = LaunchConfiguration('hyperparameters')
    return LaunchDescription([
        hyperparam_arg,
        Node(
            package='rebet',
            executable='SLAM_action_server.py',
            name='slam_action_server',
            parameters=[config_file,{}]
        ),
        Node(
            package='rebet',
            executable='bandit_adaptation_logic.py',
            name='bandit_adaptation_logic',
            parameters=[config_file,{
                'hyperparameters' : hypparams
            }]
        ),
        Node(
            package='rebet',
            executable='ID_action_server.py',
            name='identify_action_server',
            parameters=[config_file,{}]
        ),
    ])