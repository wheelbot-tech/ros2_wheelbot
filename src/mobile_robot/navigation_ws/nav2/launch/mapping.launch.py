# Description: Launch file for starting the slam_toolbox with real-time / simulation processing
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import EnvironmentVariable 
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    ld = LaunchDescription()

    # Declare launch arguments

    use_sim_time_arg = LaunchConfiguration('use_sim_time')
    param_file_arg = LaunchConfiguration('param_file')

    declare_use_sim_time_arg = DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description="Use simulation clock if true"
        )

    declare_param_file_arg = DeclareLaunchArgument(
            'param_file',
            default_value=os.path.join(
                get_package_share_directory('nav2'),
                'config',
                'mapping.yaml'),
            description='Full path to param file to load'
        )

    # Node for starting the slam_toolbox wirh real-time processing
    start_sync_slam_toolbox_node = Node(
        parameters=[
          param_file_arg,
          {'use_sim_time': use_sim_time_arg}
        ],
        package='slam_toolbox',
        executable='sync_slam_toolbox_node',
        name='slam_toolbox',
        output='screen')
    
    # Node for starting the slam_toolbox  with simulation time
    start_async_slam_toolbox_node = Node(
        parameters=[
          param_file_arg,
          {'use_sim_time': use_sim_time_arg}
        ],
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen')

    ld.add_action(declare_use_sim_time_arg)
    ld.add_action(declare_param_file_arg)
    #ld.add_action(start_sync_slam_toolbox_node)
    ld.add_action(start_sync_slam_toolbox_node)

    return ld
