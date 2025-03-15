import os

from ament_index_python.packages import get_package_share_directory

import launch
import launch_ros.actions
from launch.substitutions import  PathJoinSubstitution, LaunchConfiguration
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    use_sim_time = launch.substitutions.LaunchConfiguration('use_sim_time')
    use_stamped= launch.substitutions.LaunchConfiguration('use_stamped')
    joy_dev = launch.substitutions.LaunchConfiguration('joy_dev')
    joy_config = launch.substitutions.LaunchConfiguration('joy_config')
    joy_teleop_config = launch.substitutions.LaunchConfiguration('joy_teleop_config')

    return launch.LaunchDescription([
        launch.actions.DeclareLaunchArgument('use_sim_time', default_value='true'),
        launch.actions.DeclareLaunchArgument('joy_dev', default_value='/dev/input/js0'),
        launch.actions.DeclareLaunchArgument('joy_config', default_value='F710_sim.yaml'),
        launch.actions.DeclareLaunchArgument('use_stamped', default_value='false'),
        launch.actions.DeclareLaunchArgument('joy_teleop_config', 
            default_value=[PathJoinSubstitution([FindPackageShare("bringup_mobile"), 'config', LaunchConfiguration('joy_config')])]
        ),

        launch_ros.actions.Node(
            package='joy', executable='joy_node', name='joy_node',
            parameters=[{
                'dev': joy_dev,
                'deadzone': 0.12,
                'use_sim_time': use_sim_time,
            }]),
        launch_ros.actions.Node(
            package='teleop_twist_joy', executable='teleop_node',
            name='teleop_twist_joy_node',
            parameters=[joy_teleop_config, {'publish_stamped_twist': use_stamped}],
            remappings={('/cmd_vel', 'joy_vel')},
            ),
    ])