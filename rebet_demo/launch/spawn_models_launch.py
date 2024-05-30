from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    spawn_hydrant = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-entity', 'object_to_find_1',
            '-database', 'fire_hydrant',
            '-x', '0.07',
            '-y', '6.77',
            '-z', '-0.07'
        ],
        output='screen',
    )

    spawn_camera = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-entity', 'corner_camera_1',
            '-database', 'corner_camera',
            '-x', '6.66',
            '-y', '10.09',
            '-z', '2.84',
            '-Y', '-2.366839',
        ],
        output='screen',
    )
    
    return LaunchDescription([spawn_camera,spawn_hydrant])
