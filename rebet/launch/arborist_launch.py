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

    experiment_name_arg = DeclareLaunchArgument(
        'experiment_name',
        default_value='none',
        description='Name to give to the experiment in the CSV reporting afterwards.'
    )
    experiment_name = LaunchConfiguration('experiment_name')

    tree_name_arg = DeclareLaunchArgument(
    'tree_name',
    default_value='none',
    description='Name of behavior tree xml file'
    )
    tree_name = LaunchConfiguration('tree_name')


    task_window_arg = DeclareLaunchArgument(
    'task_qa_window',
    description='Length of QA evaluation'
    )

    power_window_arg = DeclareLaunchArgument(
    'power_qa_window',
    description='Length of QA evaluation'
    )


    task_qa_window_sub = LaunchConfiguration('task_qa_window')
    power_qa_window_sub = LaunchConfiguration('power_qa_window')


    return LaunchDescription([
        experiment_name_arg,
        tree_name_arg,
        task_window_arg,
        power_window_arg,
        Node(
            package='rebet',
            executable='arborist',
            name='arborist_node',
            parameters=[config_file,{
                'experiment_name': experiment_name,
                'bt_filename': tree_name,
                'task_qa_window': task_qa_window_sub,
                'power_qa_window': power_qa_window_sub,
            }]
        ),
    ])