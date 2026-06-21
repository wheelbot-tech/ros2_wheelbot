import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.descriptions import ParameterFile
from nav2_common.launch import ReplaceString, RewrittenYaml


DEFAULT_NAMESPACE = os.environ.get("ROS_NAMESPACE", "robot_1")


def generate_launch_description():
    package_share = get_package_share_directory("nav2")

    namespace = LaunchConfiguration("namespace")
    map_file = LaunchConfiguration("map")
    params_file = LaunchConfiguration("params_file")
    use_sim_time = LaunchConfiguration("use_sim_time")
    autostart = LaunchConfiguration("autostart")
    use_respawn = LaunchConfiguration("use_respawn")
    log_level = LaunchConfiguration("log_level")

    namespaced_params = ReplaceString(
        source_file=params_file,
        replacements={"<robot_namespace>": namespace},
    )
    configured_params = ParameterFile(
        RewrittenYaml(
            source_file=namespaced_params,
            root_key=namespace,
            param_rewrites={"use_sim_time": use_sim_time},
            convert_types=True,
        ),
        allow_substs=True,
    )

    tf_remappings = [("/tf", "tf"), ("/tf_static", "tf_static")]
    common_node_arguments = {
        "namespace": namespace,
        "output": "screen",
        "respawn": use_respawn,
        "respawn_delay": 2.0,
        "parameters": [configured_params, {"use_sim_time": use_sim_time}],
        "arguments": ["--ros-args", "--log-level", log_level],
        "remappings": tf_remappings,
    }

    map_server = Node(
        package="nav2_map_server",
        executable="map_server",
        name="map_server",
        **(common_node_arguments | {
            "parameters": [
                configured_params,
                {"use_sim_time": use_sim_time, "yaml_filename": map_file},
            ],
        }),
    )
    amcl = Node(
        package="nav2_amcl",
        executable="amcl",
        name="amcl",
        **common_node_arguments,
    )
    controller_server = Node(
        package="nav2_controller",
        executable="controller_server",
        name="controller_server",
        **(common_node_arguments | {
            "remappings": tf_remappings + [("cmd_vel", "cmd_vel_nav_raw")],
        }),
    )
    smoother_server = Node(
        package="nav2_smoother",
        executable="smoother_server",
        name="smoother_server",
        **common_node_arguments,
    )
    planner_server = Node(
        package="nav2_planner",
        executable="planner_server",
        name="planner_server",
        **common_node_arguments,
    )
    behavior_server = Node(
        package="nav2_behaviors",
        executable="behavior_server",
        name="behavior_server",
        **(common_node_arguments | {
            "remappings": tf_remappings + [("cmd_vel", "cmd_vel_nav_raw")],
        }),
    )
    bt_navigator = Node(
        package="nav2_bt_navigator",
        executable="bt_navigator",
        name="bt_navigator",
        **common_node_arguments,
    )
    waypoint_follower = Node(
        package="nav2_waypoint_follower",
        executable="waypoint_follower",
        name="waypoint_follower",
        **common_node_arguments,
    )
    velocity_smoother = Node(
        package="nav2_velocity_smoother",
        executable="velocity_smoother",
        name="velocity_smoother",
        **(common_node_arguments | {
            "remappings": tf_remappings + [
                ("cmd_vel", "cmd_vel_nav_raw"),
                ("cmd_vel_smoothed", "cmd_vel_nav_smoothed"),
            ],
        }),
    )
    collision_monitor = Node(
        package="nav2_collision_monitor",
        executable="collision_monitor",
        name="collision_monitor",
        **common_node_arguments,
    )

    localization_lifecycle_manager = Node(
        package="nav2_lifecycle_manager",
        executable="lifecycle_manager",
        namespace=namespace,
        name="lifecycle_manager_localization",
        output="screen",
        parameters=[
            {"use_sim_time": use_sim_time},
            {"autostart": autostart},
            {"node_names": ["map_server", "amcl"]},
        ],
        arguments=["--ros-args", "--log-level", log_level],
    )
    navigation_lifecycle_manager = Node(
        package="nav2_lifecycle_manager",
        executable="lifecycle_manager",
        namespace=namespace,
        name="lifecycle_manager_navigation",
        output="screen",
        parameters=[
            {"use_sim_time": use_sim_time},
            {"autostart": autostart},
            {
                "node_names": [
                    "controller_server",
                    "smoother_server",
                    "planner_server",
                    "behavior_server",
                    "velocity_smoother",
                    "collision_monitor",
                    "bt_navigator",
                    "waypoint_follower",
                ]
            },
        ],
        arguments=["--ros-args", "--log-level", log_level],
    )

    declared_arguments = [
        DeclareLaunchArgument("namespace", default_value=DEFAULT_NAMESPACE),
        DeclareLaunchArgument(
            "map",
            default_value=os.path.join(package_share, "maps", "acasa.yaml"),
        ),
        DeclareLaunchArgument(
            "params_file",
            default_value=os.path.join(package_share, "config", "nav2_wheelbot.yaml"),
        ),
        DeclareLaunchArgument("use_sim_time", default_value="false"),
        DeclareLaunchArgument("autostart", default_value="true"),
        DeclareLaunchArgument("use_respawn", default_value="true"),
        DeclareLaunchArgument("log_level", default_value="info"),
    ]

    return LaunchDescription(
        declared_arguments
        + [
            SetEnvironmentVariable("RCUTILS_LOGGING_BUFFERED_STREAM", "1"),
            map_server,
            amcl,
            controller_server,
            smoother_server,
            planner_server,
            behavior_server,
            bt_navigator,
            waypoint_follower,
            velocity_smoother,
            collision_monitor,
            localization_lifecycle_manager,
            navigation_lifecycle_manager,
        ]
    )
