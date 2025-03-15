import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction, RegisterEventHandler
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.event_handlers import OnProcessExit

NAMESPACE = os.environ.get('ROS_NAMESPACE') if 'ROS_NAMESPACE' in os.environ else 'robot1'

def generate_launch_description():
    bringup_path = get_package_share_directory("bringup_mobile")
    joystick_file = os.path.join(bringup_path, 'config', 'F710_sim.yaml')
    rviz_file = os.path.join(bringup_path, 'config', 'view.rviz')
    
    common = { 'use_sim_time': 'true', 'namespace': NAMESPACE }
    
    control_launch_args = common | {
         'hardware_plugin': 'topic_based_ros2_control/TopicBasedSystem'
    }
    
    teleoplaunch_args = common | {
        'joystick_file': joystick_file,
    }
    
    debug_launch_args = common | {
        'enable_rviz': 'true',
        'enable_foxglove': 'false',
        'rviz_file': rviz_file
    }
    
    control_layer = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    bringup_path,'launch','control_Layer.launch_2.py'
                )]), launch_arguments=control_launch_args.items())
    
    teleop_layer = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    bringup_path,'launch','teleop_Layer.launch.py'
                )]), launch_arguments=teleoplaunch_args.items())
    
    debug_layer = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    bringup_path,'launch','debug_Layer.launch.py'
                )]), launch_arguments=debug_launch_args.items())
    
    delay_teleop_layer =  TimerAction(period=6.0, actions=[teleop_layer])
    delay_debug_layer =  TimerAction(period=7.0, actions=[debug_layer])

    # Launch!
    return LaunchDescription([
        control_layer,
        delay_teleop_layer,
        delay_debug_layer,
    ])
