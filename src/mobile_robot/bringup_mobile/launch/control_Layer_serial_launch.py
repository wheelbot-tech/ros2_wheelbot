from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    declared_arguments = [
        DeclareLaunchArgument("use_sim_time", default_value="false"),
        DeclareLaunchArgument("namespace", default_value=""),
        DeclareLaunchArgument("prefix", default_value=""),
        DeclareLaunchArgument("serial_port", default_value="/dev/ttyACM0"),
        DeclareLaunchArgument("baudrate", default_value="115200"),
        DeclareLaunchArgument("command_timeout_ms", default_value="500"),
        DeclareLaunchArgument("active_modules", default_value="FR,RL"),
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
        ]
    )

    robot_description = {"robot_description": robot_description_content}

    robot_controllers = PathJoinSubstitution(
        [
            FindPackageShare("bringup_mobile"),
            "config",
            "swerve_2bot_serial_controllers.yaml",
        ]
    )

    robot_state_pub_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[robot_description, {"use_sim_time": LaunchConfiguration("use_sim_time")}],
    )

    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
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
        arguments=["joint_state_broadcaster"],
    )

    swerve_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["swerve_controller"],
    )

    delay_swerve_controller = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[swerve_controller_spawner],
        )
    )

    return LaunchDescription(
        declared_arguments
        + [
            robot_state_pub_node,
            control_node,
            joint_state_broadcaster_spawner,
            delay_swerve_controller,
        ]
    )
