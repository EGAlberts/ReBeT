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

    tree_name_arg = DeclareLaunchArgument(
    'tree_name',
    default_value='none',
    description='Name of behavior tree xml file'
    )
    tree_name = LaunchConfiguration('tree_name')


    power_window_arg = DeclareLaunchArgument(
    'power_qa_window',
    description='Length of QA evaluation'
    )

    power_qa_window_sub = LaunchConfiguration('power_qa_window')


    return LaunchDescription([
        tree_name_arg,
        power_window_arg,
        Node(
            package='rebet',
            executable='arborist',
            name='arborist_node',
            parameters=[config_file,{
                'bt_filename': tree_name,
                'power_qa_window': power_qa_window_sub,
            }]
        ),
    ])