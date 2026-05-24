import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

NAMESPACE = os.environ.get('ROS_NAMESPACE') if 'ROS_NAMESPACE' in os.environ else ''


def generate_launch_description():
    bringup_path = get_package_share_directory("bringup_mobile")
    default_controllers_file = os.path.join(
        bringup_path,
        'config',
        'swerve_2bot_serial_controllers.yaml',
    )

    declared_arguments = [
        DeclareLaunchArgument("use_sim_time", default_value="false"),
        DeclareLaunchArgument("use_stamped", default_value="true"),
        DeclareLaunchArgument("namespace", default_value=NAMESPACE),
        DeclareLaunchArgument("frame_prefix", default_value=""),
        DeclareLaunchArgument("serial_port", default_value="/dev/ttyACM0"),
        DeclareLaunchArgument("baudrate", default_value="115200"),
        DeclareLaunchArgument("command_timeout_ms", default_value="500"),
        DeclareLaunchArgument("active_modules", default_value="FR,RL"),
        DeclareLaunchArgument("controllers_file", default_value=default_controllers_file),
        DeclareLaunchArgument("joy_dev", default_value="/dev/input/js0"),
        DeclareLaunchArgument("joy_config", default_value="F710_sim.yaml"),
    ]

    joystick_file = os.path.join(bringup_path, 'config', 'F710_sim.yaml')
    #rviz_file = os.path.join(bringup_path, 'config', 'view.rviz')

    common = {
        'use_sim_time': LaunchConfiguration("use_sim_time"),
        'use_stamped': LaunchConfiguration("use_stamped"),
        'namespace': LaunchConfiguration("namespace"),
    }

    teleop_launch_args = common | {
        'joy_dev': LaunchConfiguration("joy_dev"),
        'joy_config': LaunchConfiguration("joy_config"),
    }

    tmux_launch_args = {
        'use_sim_time': LaunchConfiguration("use_sim_time"),
        'use_stamped': LaunchConfiguration("use_stamped"),
        'namespace': LaunchConfiguration("namespace"),
    }

    control_launch_args = common | {
        'frame_prefix': LaunchConfiguration("frame_prefix"),
        'serial_port': LaunchConfiguration("serial_port"),
        'baudrate': LaunchConfiguration("baudrate"),
        'command_timeout_ms': LaunchConfiguration("command_timeout_ms"),
        'active_modules': LaunchConfiguration("active_modules"),
        'controllers_file': LaunchConfiguration("controllers_file"),
    }

    #debug_launch_args = common | {
    #    'enable_rviz': 'true',
    #    'enable_foxglove': 'false',
    #    'rviz_file': rviz_file,
    #}

    control_layer = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(bringup_path, 'launch', 'control_Layer_serial_launch.py')
        ),
        launch_arguments=control_launch_args.items(),
    )

    teleop_layer = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(bringup_path, 'launch', 'teleop_Layer.launch.py')
        ),
        launch_arguments=teleop_launch_args.items(),
    )

    tmux_layer = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(bringup_path, 'launch', 'twist_mux_launch.py')
        ),
        launch_arguments=tmux_launch_args.items(),
    )

    #debug_layer = IncludeLaunchDescription(
    #    PythonLaunchDescriptionSource(
    #        os.path.join(bringup_path, 'launch', 'debug_Layer.launch.py')
    #    ),
    #    launch_arguments=debug_launch_args.items(),
    #)

    delay_teleop_layer = TimerAction(period=6.0, actions=[teleop_layer])
    delay_tmux_layer = TimerAction(period=7.0, actions=[tmux_layer])
    #delay_debug_layer = TimerAction(period=8.0, actions=[debug_layer])

    return LaunchDescription(declared_arguments + [
        control_layer,
        delay_teleop_layer,
        delay_tmux_layer,
        #delay_debug_layer,
    ])
