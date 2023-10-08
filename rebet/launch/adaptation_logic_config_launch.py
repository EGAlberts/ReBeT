from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from launch.conditions import LaunchConfigurationEquals, IfCondition
from launch.substitutions import AndSubstitution
def generate_launch_description():

    config_file = os.path.join(
    get_package_share_directory('rebet'),
    'config',
    'rebet_config.yaml'
    )

    adaptation_logic_arg = DeclareLaunchArgument(
        'adaptation_logic',
        default_value="",
        description='Adaptation manager in charge' +
                    '[bandit or reactive or random]'
    )
    
    adaptation_logic = LaunchConfiguration('adaptation_logic')

    return LaunchDescription([
        adaptation_logic_arg,
        Node(
            package='rebet',
            executable='bandit_adaptation_logic.py',
            name='bandit_adaptation_logic',
            parameters=[config_file,{}],
            condition=LaunchConfigurationEquals('adaptation_logic', 'bandit')
        ),
        Node(
            package='rebet',
            executable='random_adaptation_logic.py',
            name='random_adaptation_logic',
            parameters=[config_file,{}],
            condition=LaunchConfigurationEquals('adaptation_logic', 'random')
        ),
        Node(
            package='rebet',
            executable='reactive_adaptation_logic.py',
            name='reactive_adaptation_logic',
            parameters=[config_file,{}],
            condition=LaunchConfigurationEquals('adaptation_logic', 'reactive')
        ),

    ])