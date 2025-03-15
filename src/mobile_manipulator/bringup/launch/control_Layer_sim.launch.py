# Copyright (c) 2021 PickNik, Inc.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
#    * Redistributions of source code must retain the above copyright
#      notice, this list of conditions and the following disclaimer.
#
#    * Redistributions in binary form must reproduce the above copyright
#      notice, this list of conditions and the following disclaimer in the
#      documentation and/or other materials provided with the distribution.
#
#    * Neither the name of the {copyright_holder} nor the names of its
#      contributors may be used to endorse or promote products derived from
#      this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

#
# Author: Denis Stogl

from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterFile, ParameterValue
from launch_ros.substitutions import FindPackageShare
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, RegisterEventHandler
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import (
    AndSubstitution,
    Command,
    FindExecutable,
    LaunchConfiguration,
    NotSubstitution,
    PathJoinSubstitution,
)
from launch.event_handlers import OnProcessExit

def launch_setup(context, *args, **kwargs):
    
    # Mobile manipulator general arguments
    prefix = LaunchConfiguration("prefix")

    # Mobile robot Initialize Arguments
    namespace = LaunchConfiguration('namespace')
    hardware_plugin = LaunchConfiguration('hardware_plugin')
    use_sim_time = LaunchConfiguration('use_sim_time')

    # UR Initialize Arguments
    ur_type = LaunchConfiguration("ur_type")
    tf_prefix = LaunchConfiguration("tf_prefix")
    # UR General arguments
    use_fake_hardware = LaunchConfiguration("use_fake_hardware")


    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution([FindPackageShare("description"), "urdf", "mobile_manipulator.urdf.xacro"]),
            " ",
            "hw_interface_plugin:=", 
            hardware_plugin,
            " ",
            "namespace:=", 
            namespace,
            " ",
            "prefix:=", 
            prefix,
            " ",
            "name:=",
            "ur",
            " ",
            "ur_type:=",
            ur_type,
            " ",
            "tf_prefix:=",
            tf_prefix,
            " ",
            "use_fake_hardware:=",
            use_fake_hardware,
        ]
    )
    robot_description = {
        "robot_description": ParameterValue(value=robot_description_content, value_type=str)
    }


    ## mobile robot specific arguments
    scan_filter_config = PathJoinSubstitution([FindPackageShare("bringup_mobile"), "config", "angular_filter_scan.yaml"])
    robot_controllers = PathJoinSubstitution([FindPackageShare("bringup"), "config", "robot_sim_controllers.yaml"]) 


    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[{"use_sim_time": True}, robot_description],
    )
    
    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        #namespace=namespace,
        parameters=[robot_description, robot_controllers, {'use_sim_time': use_sim_time}],
        output="both",
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
    )
    
    swerve_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        #namespace=namespace,
        #arguments=["swerve_controller", "--controller-manager", ["/", namespace, "/controller_manager"]],
        arguments=["swerve_controller"],
    )
    FR_drive_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        #namespace=namespace,
        #arguments=["FR_drive_controller", "--controller-manager", ["/", namespace, "/controller_manager"]],
        arguments=["FR_drive_controller"],
    )
    FL_drive_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        #namespace=namespace,
        #arguments=["FL_drive_controller", "--controller-manager", ["/", namespace, "/controller_manager"]],
        arguments=["FL_drive_controller"],
    )
    RR_drive_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        #namespace=namespace,
        #arguments=["RR_drive_controller", "--controller-manager", ["/", namespace, "/controller_manager"]],
        arguments=["RR_drive_controller"],
    )
    RL_drive_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        #namespace=namespace,
        #arguments=["RL_drive_controller", "--controller-manager", ["/", namespace, "/controller_manager"]],
        arguments=["RL_drive_controller"],
    )

    # Delay start of robot_controller after `joint_state_broadcaster`
    delay_swerve_controller_spawner_after_joint_state_broadcaster_spawner = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[swerve_controller_spawner],
        )
    )
    delay_FR_drive_controller_spawner_after_swerve_controller_spawner = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=swerve_controller_spawner,
            on_exit=[FR_drive_controller_spawner],
        )
    )
    delay_FL_drive_controller_spawner_after_FR_drive_controller_spawner = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=FR_drive_controller_spawner,
            on_exit=[FL_drive_controller_spawner],
        )
    )
    delay_RR_drive_controller_spawner_after_FL_drive_controller_spawner = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=FL_drive_controller_spawner,
            on_exit=[RR_drive_controller_spawner],
        )
    )
    delay_RL_drive_controller_spawner_after_RR_drive_controller_spawner = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=RR_drive_controller_spawner,
            on_exit=[RL_drive_controller_spawner],
        )
    )
    jointstate_aggregator_node = Node(
        package='bringup_mobile',
        executable='jointstate_sim_aggregator_4',
        namespace=namespace,
        name='jointstate_sim_aggregator_4',
        output="screen",
    )
    delay_jointstate_aggregator_node_after_spawners = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=RL_drive_controller_spawner,
            on_exit=[jointstate_aggregator_node],
        )
    )
    convertor_node = Node(
        package="bringup_mobile",
        executable="jointstate_to_twist_4",
        namespace=namespace,
        name="jointstate_to_twist_4",
        output="screen",
        parameters=[{
            'prefix': LaunchConfiguration('prefix'),
            'use_stamped': LaunchConfiguration('use_stamped'),
            'wheel_radius': LaunchConfiguration('wheel_radius'),
            'wheel_drive_len': LaunchConfiguration('wheel_drive_len'),
            'drive_1_pos': LaunchConfiguration('drive_1_pos'),
            'drive_2_pos': LaunchConfiguration('drive_2_pos'),
            'drive_3_pos': LaunchConfiguration('drive_3_pos'),
            'drive_4_pos': LaunchConfiguration('drive_4_pos')
        }],
        arguments=[
            LaunchConfiguration('prefix'),
            LaunchConfiguration('use_stamped'),
            LaunchConfiguration('wheel_radius'),
            LaunchConfiguration('wheel_drive_len'),
            LaunchConfiguration('drive_1_pos'),
            LaunchConfiguration('drive_2_pos'),
            LaunchConfiguration('drive_3_pos'),
            LaunchConfiguration('drive_4_pos')
        ]
    )
    delay_convertor_node_after_spawners = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=RL_drive_controller_spawner,
            on_exit=[convertor_node],
        )
    )
    front_laser_filter_node =    Node(
            package="laser_filters",
            executable="scan_to_scan_filter_chain",
            namespace="front_lidar",
            parameters=[scan_filter_config, {'use_sim_time': use_sim_time}],
    )
    delay_front_laser_filter_node_after_spawners = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=RL_drive_controller_spawner,
            on_exit=[front_laser_filter_node],
        )
    )
    back_laser_filter_node =    Node(
            package="laser_filters",
            executable="scan_to_scan_filter_chain",
            namespace="back_lidar",
            parameters=[scan_filter_config, {'use_sim_time': use_sim_time}],
    )
    delay_back_laser_filter_node_after_spawners = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=RL_drive_controller_spawner,
            on_exit=[back_laser_filter_node],
        )
    )
    ur_joint_controller_spawner_started = Node(
        package="controller_manager",
        executable="spawner",
        #arguments=["joint_trajectory_controller", "-c", "/controller_manager"],
        arguments=["joint_trajectory_controller"],
    )
    delay_ur_joint_controller_after_spawners = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=RL_drive_controller_spawner,
            on_exit=[ur_joint_controller_spawner_started],
        )
    )
    nodes_to_start = [
        robot_state_publisher_node,
        control_node,
        joint_state_broadcaster_spawner,
        delay_swerve_controller_spawner_after_joint_state_broadcaster_spawner,
        delay_FR_drive_controller_spawner_after_swerve_controller_spawner,
        delay_FL_drive_controller_spawner_after_FR_drive_controller_spawner,
        delay_RR_drive_controller_spawner_after_FL_drive_controller_spawner,
        delay_RL_drive_controller_spawner_after_RR_drive_controller_spawner,
        delay_jointstate_aggregator_node_after_spawners,
        delay_convertor_node_after_spawners,
        delay_front_laser_filter_node_after_spawners,
        delay_back_laser_filter_node_after_spawners,
        delay_ur_joint_controller_after_spawners,

    ] 

    return nodes_to_start


def generate_launch_description():
    declared_arguments = []
    # robot specific arguments ******************************************************************

    declared_arguments.append(
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='True',
            description='Use sim time if true')
    )
    # mobile robot specific arguments ******************************************************************
    declared_arguments.append(
        DeclareLaunchArgument(
            'namespace',
            default_value='',  # 'robot1'
            description='The namespace of nodes and links')
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            'hardware_plugin',
            default_value='topic_based_ros2_control/TopicBasedSystem',
            description='Which ros2 control hardware plugin to use')
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            'prefix',
            default_value='',
            description='Prefix for the joint name')
    )
    declared_arguments.append(
         DeclareLaunchArgument(
            'wheel_radius',
            default_value= '0.0825',   # Note: default_value must be a string
            description='Drive module wheel radius')
    )
    declared_arguments.append(
         DeclareLaunchArgument(
            'wheel_drive_len',
            default_value= '0.230',   
            description='Drive module wheel separation')
    )
    declared_arguments.append(
         DeclareLaunchArgument(
            'drive_1_pos',
            default_value='FR_',
            description='Drive module 1 position')
    )
    declared_arguments.append(
         DeclareLaunchArgument(
            'drive_2_pos',
            default_value='FL_',
            description='Drive module 2 position')
    )
    declared_arguments.append(
         DeclareLaunchArgument(
            'drive_3_pos',
            default_value='RR_',
            description='Drive module 3 position')
    )
    declared_arguments.append(
         DeclareLaunchArgument(
            'drive_4_pos',
            default_value='RL_',
            description='Drive module 4 position ')
    )        
    # UR specific arguments ******************************************************************
    declared_arguments.append(
        DeclareLaunchArgument(
            "ur_type",
            default_value='ur10e',
            description="Type/series of used UR robot.",
            choices=["ur3", "ur3e", "ur5", "ur5e", "ur10", "ur10e", "ur16e", "ur20", "ur30"],
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            'tf_prefix',
            default_value='',
            description='Prefix for the joint name')
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            "use_fake_hardware",
            default_value="true",
            description="Start robot with fake hardware mirroring command to its states.",
        )
    )


    return LaunchDescription(declared_arguments + [OpaqueFunction(function=launch_setup)])