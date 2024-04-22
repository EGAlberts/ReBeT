from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument

def generate_launch_description():


    adap_period_arg = DeclareLaunchArgument(
    'adaptation_period',
    description='How often adaptation happens'
    )
    adap_period_sub = LaunchConfiguration('adaptation_period')


    tree_name = LaunchConfiguration('tree_name')
    return LaunchDescription([
        adap_period_arg,
        Node(
            package='rebet',
            executable='system_reflection.py',
            name='system_reflection_node',
            parameters=[{}]
        ),
        Node(
            package='aal',
            executable='adaptation_manager_node.py',
            name='adaptation_manager_node',
            parameters=[{'adaptation_period' : adap_period_sub}]
        ),
    ])