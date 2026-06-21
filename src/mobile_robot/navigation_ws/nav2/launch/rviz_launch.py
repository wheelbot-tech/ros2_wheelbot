# Copyright (c) 2022 Neobotix GmbH
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

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    EmitEvent,
    IncludeLaunchDescription,
    RegisterEventHandler,
)
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from nav2_common.launch import ReplaceString


DEFAULT_NAMESPACE = os.environ.get("ROS_NAMESPACE", "robot_1")


def generate_launch_description():
    package_share = get_package_share_directory("nav2")
    namespace = LaunchConfiguration("namespace")
    frame_prefix = LaunchConfiguration("frame_prefix")
    rviz_config_file = LaunchConfiguration("rviz_config")
    use_sim_time = LaunchConfiguration("use_sim_time")

    robot_view = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("description_mobile"),
                "launch",
                "view_robot_swerve.launch.py",
            )
        ),
        launch_arguments={
            "namespace": namespace,
            "frame_prefix": frame_prefix,
            "gui": "false",
            "publish_robot_state": "true",
            "use_joint_state_publisher": "true",
            "use_joint_state_publisher_gui": "false",
            "joint_states_topic": "joint_states",
            "expanded_joint_states_topic": "description_joint_states",
            "robot_description_topic": "description_robot_description",
            "tf_prefix": "",
        }.items(),
    )

    namespaced_rviz_config_file = ReplaceString(
        source_file=rviz_config_file,
        replacements={"<robot_namespace>": namespace},
    )

    rviz = Node(
        package="rviz2",
        executable="rviz2",
        namespace=namespace,
        name="nav2_rviz",
        output="screen",
        arguments=["-d", namespaced_rviz_config_file],
        parameters=[{"use_sim_time": use_sim_time}],
        remappings=[("/tf", "tf"), ("/tf_static", "tf_static")],
    )

    shutdown_when_rviz_exits = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=rviz,
            on_exit=EmitEvent(event=Shutdown(reason="RViz exited")),
        )
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("namespace", default_value=DEFAULT_NAMESPACE),
            DeclareLaunchArgument(
                "frame_prefix",
                default_value=PythonExpression(
                    ["'", namespace, "/' if '", namespace, "' != '' else ''"]
                ),
            ),
            DeclareLaunchArgument(
                "rviz_config",
                default_value=os.path.join(package_share, "rviz", "single_robot.rviz"),
            ),
            DeclareLaunchArgument("use_sim_time", default_value="false"),
            robot_view,
            rviz,
            shutdown_when_rviz_exits,
        ]
    )
