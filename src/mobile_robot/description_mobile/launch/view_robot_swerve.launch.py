# Copyright 2021 Stogl Robotics Consulting UG (haftungsbeschränkt)
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

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.conditions import IfCondition
from launch.substitutions import (
    Command,
    FindExecutable,
    LaunchConfiguration,
    PathJoinSubstitution,
    PythonExpression,
)

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Declare arguments
    declared_arguments = []
    declared_arguments.append(
        DeclareLaunchArgument(
            "description_package",
            default_value="description_mobile",
            description="Description package with robot URDF/xacro files. Usually the argument \
        is not set, it enables use of a custom description.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "description_file",
            default_value="robots/swerve_2bot_serial.urdf.xacro",
            description="URDF/XACRO description file with the robot.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "namespace",
            default_value="",
            description="Robot namespace. Leave empty for local URDF visualization.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "frame_prefix",
            default_value="",
            description="TF frame prefix used by robot_state_publisher.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "rmw_implementation",
            default_value="rmw_zenoh_cpp",
            description="RMW implementation for local monitoring nodes.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "gui",
            default_value="true",
            description="Start RViz2 automatically with this launch file.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "publish_robot_state",
            default_value="false",
            description="Start a local robot_state_publisher for offline visualization.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "use_joint_state_publisher_gui",
            default_value="false",
            description="Start joint_state_publisher_gui for offline visualization.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "use_joint_state_publisher",
            default_value="false",
            description="Start joint_state_publisher to expand mimic joints from the source JointState topic.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "tf_prefix",
            default_value="",
            description="UR-style prefix applied by xacro to TF frames and joint names, for example robot_1/.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "rviz_config",
            default_value="amr_view.rviz",
            description="RViz config from description_mobile/rviz.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "joint_states_topic",
            default_value="joint_states",
            description="Source JointState topic, relative to namespace unless absolute.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "expanded_joint_states_topic",
            default_value="description_joint_states",
            description="Expanded JointState topic consumed by robot_state_publisher.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "robot_description_topic",
            default_value="robot_description",
            description="Robot description topic used by RViz, relative to namespace.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "visualize_imu",
            default_value="true",
            description="Start the local IMU MarkerArray visualizer for RViz.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "imu_topic",
            default_value="imu/data",
            description="IMU input topic for the visualizer, relative to namespace unless absolute.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "imu_markers_topic",
            default_value="imu_markers",
            description="MarkerArray output topic for RViz, relative to namespace unless absolute.",
        )
    )

    # Initialize Arguments
    description_package = LaunchConfiguration("description_package")
    description_file = LaunchConfiguration("description_file")
    namespace = LaunchConfiguration("namespace")
    frame_prefix = LaunchConfiguration("frame_prefix")
    rmw_implementation = LaunchConfiguration("rmw_implementation")
    gui = LaunchConfiguration("gui")
    publish_robot_state = LaunchConfiguration("publish_robot_state")
    use_joint_state_publisher_gui = LaunchConfiguration("use_joint_state_publisher_gui")
    use_joint_state_publisher = LaunchConfiguration("use_joint_state_publisher")
    tf_prefix = LaunchConfiguration("tf_prefix")
    rviz_config = LaunchConfiguration("rviz_config")
    joint_states_topic = LaunchConfiguration("joint_states_topic")
    expanded_joint_states_topic = LaunchConfiguration("expanded_joint_states_topic")
    robot_description_topic_name = LaunchConfiguration("robot_description_topic")
    visualize_imu = LaunchConfiguration("visualize_imu")
    imu_topic = LaunchConfiguration("imu_topic")
    imu_markers_topic = LaunchConfiguration("imu_markers_topic")

    # Get URDF via xacro
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [FindPackageShare(description_package), "urdf", description_file]
            ),
            " ",
            "namespace:=",
            namespace,
            " ",
            "prefix:=",
            tf_prefix,
        ]
    )
    robot_description = {"robot_description": robot_description_content}
    robot_description_topic = PythonExpression(
        [
            "'/' + '",
            namespace,
            "' + '/' + '",
            robot_description_topic_name,
            "' if '",
            namespace,
            "' != '' else '/' + '",
            robot_description_topic_name,
            "'",
        ]
    )
    source_joint_states_topic = PythonExpression(
        [
            "'",
            joint_states_topic,
            "' if '",
            joint_states_topic,
            "'.startswith('/') else ('/' + '",
            namespace,
            "' + '/' + '",
            joint_states_topic,
            "' if '",
            namespace,
            "' != '' else '/' + '",
            joint_states_topic,
            "')",
        ]
    )
    tf_topic = PythonExpression(
        [
            "'/' + '",
            namespace,
            "' + '/tf' if '",
            namespace,
            "' != '' else '/tf'",
        ]
    )
    tf_static_topic = PythonExpression(
        [
            "'/' + '",
            namespace,
            "' + '/tf_static' if '",
            namespace,
            "' != '' else '/tf_static'",
        ]
    )
    tf_remappings = [
        ("/tf", tf_topic),
        ("/tf_static", tf_static_topic),
    ]
    joint_state_remappings = [
        ("joint_states", expanded_joint_states_topic),
        ("/joint_states", expanded_joint_states_topic),
    ]
    robot_description_remappings = [
        ("robot_description", robot_description_topic),
        ("/robot_description", robot_description_topic),
    ]

    rviz_config_file = PathJoinSubstitution(
        [FindPackageShare(description_package), "rviz", rviz_config]
    )

    joint_state_publisher_node = Node(
        package="joint_state_publisher",
        executable="joint_state_publisher",
        name="description_joint_state_publisher",
        namespace=namespace,
        parameters=[
            robot_description,
            {"source_list": [source_joint_states_topic]},
        ],
        remappings=joint_state_remappings + robot_description_remappings,
        condition=IfCondition(use_joint_state_publisher),
    )
    joint_state_publisher_gui_node = Node(
        package="joint_state_publisher_gui",
        executable="joint_state_publisher_gui",
        name="description_joint_state_publisher_gui",
        namespace=namespace,
        parameters=[robot_description],
        remappings=joint_state_remappings + robot_description_remappings,
        condition=IfCondition(use_joint_state_publisher_gui),
    )
    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="description_robot_state_publisher",
        namespace=namespace,
        output="both",
        parameters=[
            robot_description,
            {"frame_prefix": frame_prefix},
        ],
        remappings=tf_remappings + joint_state_remappings + robot_description_remappings,
        condition=IfCondition(publish_robot_state),
    )
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config_file],
        remappings=[
            *tf_remappings,
            *robot_description_remappings,
        ],
        condition=IfCondition(gui),
    )
    imu_motion_visualizer_node = Node(
        package="bringup_mobile",
        executable="imu_motion_visualizer",
        namespace=namespace,
        output="both",
        remappings=[
            ("imu/data", imu_topic),
            ("imu_markers", imu_markers_topic),
        ],
        condition=IfCondition(visualize_imu),
    )

    nodes = [
        SetEnvironmentVariable("RMW_IMPLEMENTATION", rmw_implementation),
        joint_state_publisher_node,
        joint_state_publisher_gui_node,
        robot_state_publisher_node,
        imu_motion_visualizer_node,
        rviz_node,
    ]

    return LaunchDescription(declared_arguments + nodes)
