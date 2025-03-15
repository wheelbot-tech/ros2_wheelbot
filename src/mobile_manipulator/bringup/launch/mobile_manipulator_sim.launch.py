import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

NAMESPACE = os.environ.get('ROS_NAMESPACE') if 'ROS_NAMESPACE' in os.environ else ''

def generate_launch_description():
    
    bringup_path = FindPackageShare("bringup")
    bringup_mobile_path = FindPackageShare("bringup_mobile")
    joystick_file = PathJoinSubstitution([bringup_mobile_path, "config", "F710_sim.yaml"])
    rviz_file = PathJoinSubstitution([bringup_path, "config", "view.rviz"])
    
    common = { 'use_sim_time': 'True', 'use_stamped': 'False', 'namespace': NAMESPACE }  
    
    control_launch_args = common | {
         'hardware_plugin': 'topic_based_ros2_control/TopicBasedSystem'
    }
    
    teleoplaunch_args = common | {
        'joystick_file': joystick_file,
    }
    
    tmux_launch_args = common

    debug_launch_args = common | {
        'enable_rviz': 'true',
        'enable_foxglove': 'false',
        'rviz_file': rviz_file
    }
    
    control_layer = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([PathJoinSubstitution([
                    bringup_path, "launch", "control_Layer_sim.launch.py"
                ])]), launch_arguments=control_launch_args.items())

    teleop_layer = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([PathJoinSubstitution([
                    bringup_mobile_path,'launch','teleop_Layer.launch.py'
                ])]), launch_arguments=teleoplaunch_args.items())
    
    tmux_layer = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([PathJoinSubstitution([
                    bringup_mobile_path,'launch','twist_mux_launch.py'
                ])]), launch_arguments=tmux_launch_args.items())
    
    debug_layer = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([PathJoinSubstitution([
                    bringup_mobile_path,'launch','debug_Layer.launch.py'
                ])]), launch_arguments=debug_launch_args.items())
    
    delay_teleop_layer =  TimerAction(period=6.0, actions=[teleop_layer])
    delay_tmux_layer =  TimerAction(period=7.0, actions=[tmux_layer])
    delay_debug_layer =  TimerAction(period=8.0, actions=[debug_layer])

    # Launch!
    return LaunchDescription([
        control_layer,
        delay_teleop_layer,
        delay_tmux_layer,
        #delay_debug_layer,
    ])
