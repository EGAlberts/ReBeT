from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from launch.conditions import LaunchConfigurationEquals

def generate_launch_description():

    config_file = os.path.join(
    get_package_share_directory('rebet'),
    'config',
    'rebet_config.yaml'
    )

    task_name_arg = DeclareLaunchArgument(
    'task_name',
    description='Which task to launch'
    )
    task_name = LaunchConfiguration('task_name')

    det_rate_arg = DeclareLaunchArgument(
    'det_rate',
    default_value = "3",
    description='detection rate'
    )
    det_rate_sub = LaunchConfiguration('det_rate')

    det_thresh_arg = DeclareLaunchArgument(
    'det_thresh',
    default_value = "1",
    description='When the mission ends'
    )
    det_thresh_sub = LaunchConfiguration('det_thresh')

    return LaunchDescription([
        task_name_arg,
        det_rate_arg,
        det_thresh_arg,
        Node(
            package='rebet',
            executable='SLAM_action_server.py',
            name='slam_action_server',
            parameters=[config_file,{}],
            condition=LaunchConfigurationEquals('task_name', 'SLAM')
        ),
        Node(
            package='rebet',
            executable='ID_action_server.py',
            name='identify_action_server',
            parameters=[config_file,{
                'pic_rate' : det_rate_sub,
                'det_threshold': det_thresh_sub
            }],
            condition=LaunchConfigurationEquals('task_name', 'ID')
        ),
    ])