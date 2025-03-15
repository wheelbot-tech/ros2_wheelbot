import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
from launch.conditions import IfCondition
from xacro import process_file




def generate_launch_description():
    # Paths to places
    pkg_wheelbot_isaac_path = get_package_share_directory('isaac_sim')
    #isaac_install_path = "home/george/.local/share/ov/pkg/isaac-sim-4.2.0"
    isaac_install_path = "/home/george/isaacsim"
    isaac_python_launcher_path = os.path.join(isaac_install_path, "python.sh")
    isaac_custom_launch_script = os.path.join(pkg_wheelbot_isaac_path, "tools", "wheelbot_ur_script.py")
    full_path_to_world = PathJoinSubstitution([pkg_wheelbot_isaac_path, 'isaac_worlds', LaunchConfiguration('world_name')])
    full_path_to_mobile_robot = PathJoinSubstitution([pkg_wheelbot_isaac_path, 'isaac_description', LaunchConfiguration('mobile_robot_name')])
    full_path_to_manipulator_robot = PathJoinSubstitution([pkg_wheelbot_isaac_path, 'isaac_description/UR10', LaunchConfiguration('manipulator_robot_name')])
    full_path_to_gripper = PathJoinSubstitution([pkg_wheelbot_isaac_path, 'isaac_description/Robotiq/2F-140', LaunchConfiguration('gripper_name')])

    # Arguments
    world_name = DeclareLaunchArgument(
        'world_name',
        default_value='cylinder_garden2.usda',
        description='Name of the world to launch',
        choices=[world for world in os.listdir(os.path.join(pkg_wheelbot_isaac_path, 'isaac_worlds')) if world.endswith(('.usd', '.usda'))]
    )
    mobile_robot_name = DeclareLaunchArgument(
        'mobile_robot_name',
        default_value='swerveBOT_910_UR10_ROS.usda',
        description='Name of the mobile robot to spawn',
        choices=[robot for robot in os.listdir(os.path.join(pkg_wheelbot_isaac_path, 'isaac_description')) if robot.endswith(('.usd', '.usda'))]
    )
    manipulator_robot_name = DeclareLaunchArgument(
        'manipulator_robot_name',
        default_value= 'ur10_long_suction.usd',   #'ur10e_robotiq2f-140_ROS.usd'
        description='Name of the manipulator robot to spawn',
        choices=[robot for robot in os.listdir(os.path.join(pkg_wheelbot_isaac_path, 'isaac_description/UR10')) if robot.endswith(('.usd', '.usda'))]
    )
    gripper_name = DeclareLaunchArgument(
        'gripper_name',
        default_value= 'Robotiq_2F_140_config.usd',       
        description='Name of the Robotiq gripper to spawn',
        choices=[robot for robot in os.listdir(os.path.join(pkg_wheelbot_isaac_path, 'isaac_description/Robotiq/2F-140')) if robot.endswith(('.usd', '.usda'))]
    )
    headless = DeclareLaunchArgument(
        'headless',
        default_value='False',
        description='Run Isaac sim headless',
    )
    renderer = DeclareLaunchArgument(
        'renderer',
        default_value='RayTracedLighting',
        description='Renderer to use',
        choices=["RayTracedLighting", "PathTracing"]
    )
    verbose = DeclareLaunchArgument(
        'verbose',
        default_value='True',
        description='Show Isaac sim output',
    )
    return LaunchDescription([
        world_name,
        mobile_robot_name,
        manipulator_robot_name,
        gripper_name,
        headless,
        renderer,
        verbose,
    
        ExecuteProcess(
            cmd = [
                isaac_python_launcher_path,
                isaac_custom_launch_script,
                '--world_file', full_path_to_world,
                '--mobile_robot_file', full_path_to_mobile_robot,
                '--manipulator_robot_file', full_path_to_manipulator_robot,
                '--gripper_file', full_path_to_gripper,
                '--headless', LaunchConfiguration('headless'),
                '--renderer', LaunchConfiguration('renderer')
            ],
            shell = LaunchConfiguration('verbose'),
            output = "screen",

        ),
    ])
