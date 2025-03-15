#!/usr/bin/env python3
# Copyright 2020 Gaitech Korea Co., Ltd.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

# Author: Brighten Lee

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import  PathJoinSubstitution, LaunchConfiguration
from launch_ros.actions import Node
from launch.conditions import IfCondition, UnlessCondition


def generate_launch_description():
    bringup_path = get_package_share_directory("bringup_mobile")
    default_config_locks = os.path.join(bringup_path, 'config', 'twist_mux_locks.yaml')
    default_config_topics = os.path.join(bringup_path,'config', 'twist_mux_topics.yaml')
    default_config_joystick = os.path.join(bringup_path, 'config', 'joystick.yaml')
    use_sim_time = LaunchConfiguration('use_sim_time')
    use_stamped = LaunchConfiguration('use_stamped')  

 
    twist_mux_node = Node(
        package='twist_mux',
        executable='twist_mux',
        output='screen',
        remappings=[('/cmd_vel_out', LaunchConfiguration('cmd_vel_out_stamped'))],
        parameters=[
            {'use_sim_time': use_sim_time},
            {"use_stamped": use_stamped},
            LaunchConfiguration('config_locks'),
            LaunchConfiguration('config_topics')],
        condition=IfCondition(use_stamped)
    )
    twist_unstamped_mux_node = Node(
        package='twist_mux',
        executable='twist_mux',
        output='screen',
        remappings=[('/cmd_vel_out', LaunchConfiguration('cmd_vel_out_unstamped'))],
        parameters=[
            {'use_sim_time': use_sim_time},
            {"use_stamped": use_stamped},
            LaunchConfiguration('config_locks'),
            LaunchConfiguration('config_topics')],
        condition=UnlessCondition(use_stamped)
    )
    twist_marker_node = Node(
        package='twist_mux',
        executable='twist_marker',
        output='screen',
        remappings=[('/twist', LaunchConfiguration('cmd_vel_out_stamped'))],
        parameters=[{
            'use_sim_time': use_sim_time,
            'use_stamped': use_stamped,
            'frame_id': 'base_link',
            'scale': 1.0,
            'vertical_position': 2.0}],
        condition=IfCondition(use_stamped)
    )
    twist_unstamped_marker_node = Node(
        package='twist_mux',
        executable='twist_marker',
        output='screen',
        remappings=[('/twist', LaunchConfiguration('cmd_vel_out_unstamped'))],
        parameters=[{
            'use_sim_time': use_sim_time,
            'use_stamped': use_stamped,
            'frame_id': 'base_link',
            'scale': 1.0,
            'vertical_position': 2.0}],
        condition=UnlessCondition(use_stamped)
    )
    twist_joystick_node = Node(
        package='twist_mux',
        executable='joystick_relay.py',
        output='screen',
        remappings={('joy_vel_in', 'input_joy/cmd_vel'),
                    ('joy_vel_out', 'joy_vel')},
        parameters=[
            {'use_sim_time': use_sim_time},
            LaunchConfiguration('config_joy')]
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_stamped',
            default_value='false',
            description='Use TwistStamped cmd_vel Options: true/false'),
        DeclareLaunchArgument(
            'config_locks',
            default_value=default_config_locks,
            description='Default locks config file'),
        DeclareLaunchArgument(
            'config_topics',
            default_value=default_config_topics,
            description='Default topics config file'),
        DeclareLaunchArgument(
            'config_joy',
            default_value=default_config_joystick,
            description='Default joystick config file'),
        DeclareLaunchArgument(
            'controller_prefix',
            default_value='swerve_controller',
            description='Prefix for the controller namespace'),
        DeclareLaunchArgument(
            'cmd_vel_out_stamped',
            default_value=[LaunchConfiguration('controller_prefix'), '/cmd_vel'],
            description='cmd vel output topic'),
        DeclareLaunchArgument(
            'cmd_vel_out_unstamped',
            default_value=[LaunchConfiguration('controller_prefix'), '/cmd_vel_unstamped'],
            description='cmd vel output topic'),
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation time'),
        
        twist_mux_node,
        twist_unstamped_mux_node,
        #twist_marker_node,
        #twist_unstamped_marker_node,
        #twist_joystick_node
    ])
