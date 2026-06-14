import os
import tempfile

import yaml

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    OpaqueFunction,
    RegisterEventHandler,
    SetLaunchConfiguration,
)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit, OnShutdown
from launch.logging import get_logger
from launch.substitutions import (
    Command,
    FindExecutable,
    LaunchConfiguration,
    PathJoinSubstitution,
    PythonExpression,
)
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


MODULE_TO_WHEEL = {
    "FL": "front_left",
    "FR": "front_right",
    "RL": "rear_left",
    "RR": "rear_right",
}


def _parse_active_modules(value):
    modules = []
    for item in value.replace(",", " ").split():
        module = item.rstrip("_").upper()
        if module not in MODULE_TO_WHEEL:
            raise RuntimeError(
                f"Invalid active_modules entry '{item}'. "
                "Expected FR, FL, RR, or RL."
            )
        if module in modules:
            raise RuntimeError(
                f"Duplicate active_modules entry '{item}'."
            )
        modules.append(module)

    if not modules:
        raise RuntimeError("active_modules must contain at least one module.")
    return modules


def _active_wheels_from_modules(value):
    return [MODULE_TO_WHEEL[module] for module in _parse_active_modules(value)]


def _find_swerve_controller_parameters(controllers):
    if not isinstance(controllers, dict):
        raise RuntimeError('controllers_file root must be a YAML mapping.')

    matches = []
    for node_name, node_config in controllers.items():
        normalized_name = str(node_name).rstrip('/')
        if normalized_name.split('/')[-1] != 'swerve_controller':
            continue
        if not isinstance(node_config, dict):
            continue
        parameters = node_config.get('ros__parameters')
        if isinstance(parameters, dict):
            matches.append((node_name, parameters))

    if not matches:
        raise RuntimeError(
            'controllers_file must contain a swerve_controller node with '
            'ros__parameters.'
        )
    if len(matches) > 1:
        matched_names = ', '.join(str(name) for name, _ in matches)
        raise RuntimeError(
            'controllers_file contains multiple swerve_controller parameter '
            f'sections: {matched_names}.'
        )
    return matches[0][1]


def _prepare_controllers_file(context):
    active_modules = LaunchConfiguration("active_modules").perform(context)
    modules = _parse_active_modules(active_modules)
    controllers_file = LaunchConfiguration("controllers_file").perform(context)

    try:
        with open(controllers_file, "r", encoding="utf-8") as stream:
            controllers = yaml.safe_load(stream)
    except (OSError, yaml.YAMLError) as error:
        raise RuntimeError(
            f"Cannot load controllers_file '{controllers_file}': {error}"
        ) from error

    try:
        controller_parameters = _find_swerve_controller_parameters(controllers)
    except RuntimeError as error:
        raise RuntimeError(
            f"Invalid controllers_file '{controllers_file}': {error}"
        ) from error

    active_wheels = _active_wheels_from_modules(active_modules)
    controller_parameters["active_wheels"] = active_wheels

    with tempfile.NamedTemporaryFile(
        mode="w",
        prefix="wheelbot_swerve_controllers_",
        suffix=".yaml",
        delete=False,
        encoding="utf-8",
    ) as stream:
        yaml.safe_dump(controllers, stream, sort_keys=False)
        generated_file = stream.name

    get_logger("control_Layer_serial_launch").info(
        "Derived active_wheels=%s from active_modules=%s",
        active_wheels,
        modules,
    )
    return [
        SetLaunchConfiguration("resolved_controllers_file", generated_file)
    ]


def _cleanup_controllers_file(event, context):
    del event
    generated_file = context.launch_configurations.get(
        "resolved_controllers_file", ""
    )
    if generated_file:
        try:
            os.unlink(generated_file)
        except FileNotFoundError:
            pass


def generate_launch_description():
    declared_arguments = [
        DeclareLaunchArgument("use_sim_time", default_value="false"),
        DeclareLaunchArgument("namespace", default_value=""),
        DeclareLaunchArgument("frame_prefix", default_value=""),
        DeclareLaunchArgument("prefix", default_value=""),
        DeclareLaunchArgument("serial_port", default_value="/dev/ttyACM0"),
        DeclareLaunchArgument("baudrate", default_value="115200"),
        DeclareLaunchArgument("command_timeout_ms", default_value="500"),
        DeclareLaunchArgument("state_timeout_ms", default_value="500"),
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
            "state_timeout_ms:=",
            LaunchConfiguration("state_timeout_ms"),
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

    robot_controllers = LaunchConfiguration("resolved_controllers_file")
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
    cleanup_controllers_file = RegisterEventHandler(
        OnShutdown(on_shutdown=_cleanup_controllers_file)
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
            False, False, False,
            True, True, False,
            False, False, False,
            False, False, False,
        ],
        "odom0_queue_size": 10,
        "odom0_nodelay": True,
        "odom0_differential": False,
        "odom0_relative": False,
        "imu0": "imu/data",
        "imu0_config": [
            False, False, False,
            True, True, True,
            False, False, False,
            True, True, True,
            False, False, False,
        ],
        "imu0_queue_size": 50,
        "imu0_nodelay": True,
        "imu0_differential": False,
        "imu0_relative": True,
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
            OpaqueFunction(function=_prepare_controllers_file),
            cleanup_controllers_file,
            robot_state_pub_node,
            control_node,
            joint_state_broadcaster_spawner,
            delay_swerve_controller,
            ekf_node,
        ]
    )
