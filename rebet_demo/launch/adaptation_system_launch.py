from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='rebet_demo',
            executable='system_reflection.py',
            name='system_reflection_node',
        ),
        Node(
            package='aal',
            executable='adaptation_layer',
            name='adaptation_layer_node',
        ),
    ])