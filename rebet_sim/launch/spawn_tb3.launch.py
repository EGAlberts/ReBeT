import os

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
from launch.launch_description_sources import AnyLaunchDescriptionSource
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch.actions import DeclareLaunchArgument, OpaqueFunction
import numpy as np
import random

def get_bounding_box(pose, size):
    min_x = pose[0] - size[0] / 2.0
    max_x = pose[0] + size[0] / 2.0
    min_y = pose[1] - size[1] / 2.0
    max_y = pose[1] + size[1] / 2.0
    return (min_x, max_x, min_y, max_y)

def is_point_inside_boxes(x, y, boxes):
    for box in boxes:
        min_x, max_x, min_y, max_y = get_bounding_box(box['pose'], box['size'])
        if min_x < x < max_x and min_y < y < max_y:
            return True
    return False

sparse_world_cardboard_boxes = [
    {'pose': (1.02883, 0.542919, 0.15), 'size': (0.5, 0.4, 0.3)},
    {'pose': (-1.49971, 1.59531, 0.075), 'size': (0.25, 0.2, 0.15)},
    {'pose': (0.041545, 1.67898, 0.075), 'size': (0.25, 0.2, 0.15)}
    # Add more cardboard boxes as needed
]

def launch_setup(context, *args, **kwargs):



    launch_file_dir = os.path.join(get_package_share_directory('turtlebot3_gazebo'), 'launch')

    use_sim_time = LaunchConfiguration('use_sim_time', default='true')


    gui = LaunchConfiguration('gui')
    seed = LaunchConfiguration('myseed')

    seed_index = int(seed.perform(context))

    np.random.seed(1337 * seed_index)
    
    tb3_path = get_package_share_directory('turtlebot3_gazebo')
    tb3_launch_path = os.path.join(
        tb3_path, 'launch', 'turtlebot3_world.launch.py')
    
    this_path = get_package_share_directory('rebet_sim')
    custom_world_launch_path = os.path.join(
        this_path, 'launch', 'custom_world.launch.py')
    
    fh_x_random, fh_y_random = np.random.uniform(low=-1.8, high=1.8, size=2)

    while(is_point_inside_boxes(fh_x_random,fh_y_random, sparse_world_cardboard_boxes)):
      fh_x_random, fh_y_random = np.random.uniform(low=-1.8, high=1.8, size=2)

    fire_hydrant_object = [{'pose': (fh_x_random, fh_y_random, -0.14), 'size': (0.5, 0.4, 0.3)}]

    # random.seed(1337)

    tb_x_random,tb_y_random = np.random.uniform(low=-1.7, high=1.7, size=2)

    while(is_point_inside_boxes(tb_x_random,tb_y_random, (sparse_world_cardboard_boxes+fire_hydrant_object))):
      tb__x_random, tb_y_random = np.random.uniform(low=-1.7, high=1.7, size=2) 
    

    box_x_pose = LaunchConfiguration('box_x_pose', default=str(fh_x_random))
    box_y_pose = LaunchConfiguration('box_y_pose', default=str(fh_y_random))

    tb_x_pose = LaunchConfiguration('x_pose', default=str(tb_x_random))
    tb_y_pose = LaunchConfiguration('y_pose', default=str(tb_y_random))

    custom_world =  IncludeLaunchDescription(
        AnyLaunchDescriptionSource(custom_world_launch_path),
        launch_arguments={
           'gui': gui,
        }.items())

    model_name = 'fire_hydrant_small'
    spawn_box = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-entity', 'object_to_find_1',
            '-database', model_name,
            '-x', box_x_pose,
            '-y', box_y_pose,
            '-z', '-0.14'
        ],
        output='screen',
    )

    spawn_turtlebot_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(launch_file_dir, 'spawn_turtlebot3.launch.py')
        ),
        launch_arguments={
            'x_pose': tb_x_pose,
            'y_pose': tb_y_pose
        }.items()
    )

    robot_state_publisher_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(launch_file_dir, 'robot_state_publisher.launch.py')
        ),
        launch_arguments={'use_sim_time': use_sim_time}.items()
    )

    return [custom_world,spawn_box,spawn_turtlebot_cmd, robot_state_publisher_cmd]





def generate_launch_description():
    
    gui_arg = DeclareLaunchArgument(
        'gui',
        default_value='true',
        description='Run with gui (true/false)')

    seed_arg = DeclareLaunchArgument(
        'myseed',
        default_value='0',
        description='seed')


    return LaunchDescription([
        seed_arg,
        gui_arg,
        OpaqueFunction(function=launch_setup),
    ])
