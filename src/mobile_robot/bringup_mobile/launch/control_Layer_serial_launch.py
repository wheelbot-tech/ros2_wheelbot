from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
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
    declared_arguments = [
        DeclareLaunchArgument("use_sim_time", default_value="false"),
        DeclareLaunchArgument("namespace", default_value=""),
        DeclareLaunchArgument("frame_prefix", default_value=""),
        DeclareLaunchArgument("prefix", default_value=""),
        DeclareLaunchArgument("serial_port", default_value="/dev/ttyACM0"),
        DeclareLaunchArgument("baudrate", default_value="115200"),
        DeclareLaunchArgument("command_timeout_ms", default_value="500"),
        DeclareLaunchArgument("active_modules", default_value="FR,RL"),
        DeclareLaunchArgument("enable_odom_fusion", default_value="true"),
        DeclareLaunchArgument(
            "controllers_file",
            default_value=PathJoinSubstitution(
                [
                    FindPackageShare("bringup_mobile"),
                    "config",
                    "swerve_2bot_serial_controllers.yaml",
                ]
            ),
        ),
        DeclareLaunchArgument(
            "ekf_config_file",
            default_value=PathJoinSubstitution(
                [
                    FindPackageShare("bringup_mobile"),
                    "config",
                    "ekf_swerve_imu.yaml",
                ]
            ),
        ),
    ]

    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [
                    FindPackageShare("description_mobile"),
                    "urdf",
                    "robots",
                    "swerve_2bot_serial.urdf.xacro",
                ]
            ),
            " ",
            "namespace:=",
            LaunchConfiguration("namespace"),
            " ",
            "prefix:=",
            LaunchConfiguration("prefix"),
            " ",
            "serial_port:=",
            LaunchConfiguration("serial_port"),
            " ",
            "baudrate:=",
            LaunchConfiguration("baudrate"),
            " ",
            "command_timeout_ms:=",
            LaunchConfiguration("command_timeout_ms"),
            " ",
            "active_modules:=",
            LaunchConfiguration("active_modules"),
            " ",
            "imu_topic:=",
            PythonExpression(
                [
                    "'/' + '",
                    LaunchConfiguration("namespace"),
                    "' + '/imu/data' if '",
                    LaunchConfiguration("namespace"),
                    "' != '' else '/imu/data'",
                ]
            ),
            " ",
            "imu_frame_id:=",
            PythonExpression(
                [
                    "'",
                    LaunchConfiguration("frame_prefix"),
                    "' + 'imu_link' if '",
                    LaunchConfiguration("frame_prefix"),
                    "' != '' else 'imu_link'",
                ]
            ),
        ]
    )

    robot_description = {"robot_description": robot_description_content}
    controller_manager_name = PythonExpression(
        [
            "'/' + '",
            LaunchConfiguration("namespace"),
            "' + '/controller_manager' if '",
            LaunchConfiguration("namespace"),
            "' != '' else '/controller_manager'",
        ]
    )

    robot_controllers = LaunchConfiguration("controllers_file")
    ekf_config_file = LaunchConfiguration("ekf_config_file")
    tf_topic = PythonExpression(
        [
            "'/' + '",
            LaunchConfiguration("namespace"),
            "' + '/tf' if '",
            LaunchConfiguration("namespace"),
            "' != '' else '/tf'",
        ]
    )
    tf_static_topic = PythonExpression(
        [
            "'/' + '",
            LaunchConfiguration("namespace"),
            "' + '/tf_static' if '",
            LaunchConfiguration("namespace"),
            "' != '' else '/tf_static'",
        ]
    )
    tf_remappings = [
        ("/tf", tf_topic),
        ("/tf_static", tf_static_topic),
    ]

    robot_state_pub_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        namespace=LaunchConfiguration("namespace"),
        output="both",
        parameters=[
            robot_description,
            {
                "use_sim_time": LaunchConfiguration("use_sim_time"),
                "frame_prefix": LaunchConfiguration("frame_prefix"),
            },
        ],
        remappings=tf_remappings,
    )

    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        namespace=LaunchConfiguration("namespace"),
        parameters=[
            robot_description,
            robot_controllers,
            {"use_sim_time": LaunchConfiguration("use_sim_time")},
        ],
        output="both",
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        namespace=LaunchConfiguration("namespace"),
        arguments=[
            "joint_state_broadcaster",
            "--controller-manager",
            controller_manager_name,
        ],
    )

    swerve_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        namespace=LaunchConfiguration("namespace"),
        arguments=[
            "swerve_controller",
            "--controller-manager",
            controller_manager_name,
        ],
    )

    delay_swerve_controller = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[swerve_controller_spawner],
        )
    )

    odom_frame = PythonExpression(
        [
            "'",
            LaunchConfiguration("frame_prefix"),
            "' + 'odom' if '",
            LaunchConfiguration("frame_prefix"),
            "' != '' else 'odom'",
        ]
    )
    base_footprint_frame = PythonExpression(
        [
            "'",
            LaunchConfiguration("frame_prefix"),
            "' + 'base_footprint' if '",
            LaunchConfiguration("frame_prefix"),
            "' != '' else 'base_footprint'",
        ]
    )
    ekf_default_parameters = {
        "frequency": 30.0,
        "sensor_timeout": 0.2,
        "two_d_mode": True,
        "publish_tf": True,
        "publish_acceleration": False,
        "print_diagnostics": False,
        "map_frame": "map",
        "odom0": "swerve_controller/odom",
        "odom0_config": [
            True, True, False,
            False, False, True,
            True, True, False,
            False, False, True,
            False, False, False,
        ],
        "odom0_queue_size": 10,
        "odom0_nodelay": True,
        "odom0_differential": False,
        "odom0_relative": False,
        "imu0": "imu/data",
        "imu0_config": [
            False, False, False,
            False, False, False,
            False, False, False,
            False, False, True,
            False, False, False,
        ],
        "imu0_queue_size": 50,
        "imu0_nodelay": True,
        "imu0_differential": False,
        "imu0_relative": False,
        "imu0_remove_gravitational_acceleration": False,
    }

    ekf_node = Node(
        package="robot_localization",
        executable="ekf_node",
        name="ekf_filter_node",
        namespace=LaunchConfiguration("namespace"),
        output="both",
        condition=IfCondition(LaunchConfiguration("enable_odom_fusion")),
        parameters=[
            ekf_config_file,
            ekf_default_parameters,
            {
                "use_sim_time": LaunchConfiguration("use_sim_time"),
                "odom_frame": odom_frame,
                "world_frame": odom_frame,
                "base_link_frame": base_footprint_frame,
            },
        ],
        remappings=[
            *tf_remappings,
            ("odometry/filtered", "odom"),
        ],
    )

    return LaunchDescription(
        declared_arguments
        + [
            robot_state_pub_node,
            control_node,
            joint_state_broadcaster_spawner,
            delay_swerve_controller,
            ekf_node,
        ]
    )
