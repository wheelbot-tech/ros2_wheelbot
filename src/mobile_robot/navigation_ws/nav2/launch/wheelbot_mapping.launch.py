from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent, RegisterEventHandler
from launch.events import matches_action
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.actions import LifecycleNode, Node
from launch_ros.event_handlers import OnStateTransition
from launch_ros.events.lifecycle import ChangeState
from launch_ros.substitutions import FindPackageShare
from lifecycle_msgs.msg import Transition


def generate_launch_description():
    namespace = LaunchConfiguration("namespace")
    frame_prefix = LaunchConfiguration("frame_prefix")
    scan_topic = LaunchConfiguration("scan_topic")
    params_file = LaunchConfiguration("params_file")
    use_sim_time = LaunchConfiguration("use_sim_time")

    tf_topic = PythonExpression(
        ["'/' + '", namespace, "' + '/tf' if '", namespace, "' != '' else '/tf'"]
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
    map_topic = PythonExpression(
        ["'/' + '", namespace, "' + '/map' if '", namespace, "' != '' else '/map'"]
    )
    map_updates_topic = PythonExpression(
        [
            "'/' + '",
            namespace,
            "' + '/map_updates' if '",
            namespace,
            "' != '' else '/map_updates'",
        ]
    )

    slam_toolbox = LifecycleNode(
        package="slam_toolbox",
        executable="async_slam_toolbox_node",
        name="slam_toolbox",
        namespace=namespace,
        output="screen",
        parameters=[
            params_file,
            {
                "use_sim_time": use_sim_time,
                "odom_frame": PythonExpression(["'", frame_prefix, "' + 'odom'"]),
                "map_frame": PythonExpression(["'", frame_prefix, "' + 'map'"]),
                "base_frame": PythonExpression(
                    ["'", frame_prefix, "' + 'base_footprint'"]
                ),
                "scan_topic": scan_topic,
            },
        ],
        remappings=[
            ("/tf", tf_topic),
            ("/tf_static", tf_static_topic),
            ("/map", map_topic),
            ("/map_updates", map_updates_topic),
        ],
    )

    configure_slam_toolbox = EmitEvent(
        event=ChangeState(
            lifecycle_node_matcher=matches_action(slam_toolbox),
            transition_id=Transition.TRANSITION_CONFIGURE,
        )
    )

    activate_slam_toolbox = RegisterEventHandler(
        OnStateTransition(
            target_lifecycle_node=slam_toolbox,
            start_state="configuring",
            goal_state="inactive",
            entities=[
                EmitEvent(
                    event=ChangeState(
                        lifecycle_node_matcher=matches_action(slam_toolbox),
                        transition_id=Transition.TRANSITION_ACTIVATE,
                    )
                )
            ],
        )
    )

    map_saver = Node(
        package="nav2_map_server",
        executable="map_saver_server",
        name="map_saver",
        namespace=namespace,
        output="screen",
        parameters=[
            {
                "use_sim_time": use_sim_time,
                "save_map_timeout": 10.0,
                "free_thresh_default": 0.25,
                "occupied_thresh_default": 0.65,
            }
        ],
    )

    lifecycle_manager = Node(
        package="nav2_lifecycle_manager",
        executable="lifecycle_manager",
        name="mapping_lifecycle_manager",
        namespace=namespace,
        output="screen",
        parameters=[
            {
                "use_sim_time": use_sim_time,
                "autostart": True,
                "node_names": ["map_saver"],
            }
        ],
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("namespace", default_value="robot_1"),
            DeclareLaunchArgument("frame_prefix", default_value="robot_1/"),
            DeclareLaunchArgument(
                "scan_topic", default_value="/robot_1/front_lidar/scan"
            ),
            DeclareLaunchArgument("use_sim_time", default_value="false"),
            DeclareLaunchArgument(
                "params_file",
                default_value=PathJoinSubstitution(
                    [FindPackageShare("nav2"), "config", "wheelbot_mapping.yaml"]
                ),
            ),
            slam_toolbox,
            configure_slam_toolbox,
            activate_slam_toolbox,
            map_saver,
            lifecycle_manager,
        ]
    )
