import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource

NAMESPACE = os.environ.get('ROS_NAMESPACE') if 'ROS_NAMESPACE' in os.environ else ''


def generate_launch_description():
    bringup_path = get_package_share_directory("bringup_mobile")
    joystick_file = os.path.join(bringup_path, 'config', 'F710_sim.yaml')
    #rviz_file = os.path.join(bringup_path, 'config', 'view.rviz')

    common = {
        'use_sim_time': 'false',
        'use_stamped': 'true',
        'namespace': NAMESPACE,
    }

    teleop_launch_args = common | {
        'joystick_file': joystick_file,
    }

    tmux_launch_args = {'use_sim_time': 'false', 'use_stamped': 'true'}

    #debug_launch_args = common | {
    #    'enable_rviz': 'true',
    #    'enable_foxglove': 'false',
    #    'rviz_file': rviz_file,
    #}

    control_layer = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(bringup_path, 'launch', 'control_Layer_serial_launch.py')
        )
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

    return LaunchDescription([
        control_layer,
        delay_teleop_layer,
        delay_tmux_layer,
        #delay_debug_layer,
    ])
