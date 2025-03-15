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
    isaac_install_path = "/home/george/isaacsim"
    #isaac_install_path = "home/george/.local/share/ov/pkg/isaac-sim-4.2.0"
    isaac_python_launcher_path = os.path.join(isaac_install_path, "python.sh")
    isaac_custom_launch_script = os.path.join(pkg_wheelbot_isaac_path, "tools", "wheelbot_script.py")
    full_path_to_world = PathJoinSubstitution([pkg_wheelbot_isaac_path, 'isaac_worlds', LaunchConfiguration('world_name')])
    full_path_to_robot = PathJoinSubstitution([pkg_wheelbot_isaac_path, 'isaac_description', LaunchConfiguration('robot_name')])

    # Environment variables
    prev_ld_library_path = os.environ.get("LD_LIBRARY_PATH", "")
    ld_library_path_env_var = prev_ld_library_path + ":" + isaac_install_path + "/exts/omni.isaac.ros2_bridge/humble/lib"
    rmw_implementation_env_var = 'rmw_cyclonedds_cpp'

    # Arguments
    world_name = DeclareLaunchArgument(
        'world_name',
        default_value='cylinder_garden2.usda',
        description='Name of the world to launch',
        choices=[world for world in os.listdir(os.path.join(pkg_wheelbot_isaac_path, 'isaac_worlds')) if world.endswith(('.usd', '.usda'))]
    )
    robot_name = DeclareLaunchArgument(
        'robot_name',
        default_value='swerveBOT_910.usda',
        description='Name of the robot to spawn',
        choices=[robot for robot in os.listdir(os.path.join(pkg_wheelbot_isaac_path, 'isaac_description')) if robot.endswith(('.usd', '.usda'))]
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
        robot_name,
        headless,
        renderer,
        verbose,
    
        ExecuteProcess(
            cmd = [
                isaac_python_launcher_path,
                isaac_custom_launch_script,
                '--world_file', full_path_to_world,
                '--robot_file', full_path_to_robot,
                '--headless', LaunchConfiguration('headless'),
                '--renderer', LaunchConfiguration('renderer')
            ],
            shell = LaunchConfiguration('verbose'),
            output = "screen",
            additional_env = {
                #"RMW_IMPLEMENTATION": rmw_implementation_env_var
                #"LD_LIBRARY_PATH": ld_library_path_env_var
            }
        ),
    ])
