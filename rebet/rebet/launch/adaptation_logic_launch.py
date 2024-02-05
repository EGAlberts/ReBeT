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

    hyperparam_arg = DeclareLaunchArgument(
    'hyperparameters',
    description='Hyperparameters to the bandit',
    default_value = '[""]',
    )

    bandit_name_arg = DeclareLaunchArgument(
    'bandit_name',
    default_value = "",
    description='Which bandit'
    )
    
    adaptation_logic_arg = DeclareLaunchArgument(
        'adaptation_logic',
        default_value="random",
        description='Adaptation manager in charge' +
                    '[bandit or reactive or random]'
    )
    bandit_name = LaunchConfiguration('bandit_name')

    hypparams = LaunchConfiguration('hyperparameters')
    adaptation_logic = LaunchConfiguration('adaptation_logic')

    return LaunchDescription([
        bandit_name_arg,
        hyperparam_arg,
        adaptation_logic_arg,
        Node(
            package='rebet',
            executable='bandit_adaptation_logic.py',
            name='bandit_adaptation_logic',
            parameters=[{
                'hyperparameters' : hypparams,
                'bandit_name' : bandit_name
            }],
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