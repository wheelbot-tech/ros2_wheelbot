from launch import LaunchDescription
from launch.actions import ExecuteProcess, DeclareLaunchArgument, RegisterEventHandler, LogInfo, EmitEvent
from launch.conditions import IfCondition, UnlessCondition
from launch.event_handlers import OnProcessExit
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution, LaunchConfiguration
from launch_ros.descriptions import ParameterValue
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.events import Shutdown

def generate_launch_description():

    # Declare arguments
    declared_arguments = []
    declared_arguments.append(
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use sim time if true')
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            'namespace',
            default_value='robot1',
            description='The namespace of nodes and links')
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            'hardware_plugin',
            default_value='"topic_based_ros2_control/TopicBasedSystem"',
            description='Which ros2 control hardware plugin to use')
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            'prefix',
            default_value='',
            description='Prefix for the joint name')
    )
    declared_arguments.append(
         DeclareLaunchArgument(
            'wheel_radius',
            default_value= '0.0825',   # Note: default_value must be a string
            description='Drive module wheel radius')
    )
    declared_arguments.append(
         DeclareLaunchArgument(
            'wheel_drive_len',
            default_value= '0.23',   
            description='Drive module wheel separation')
    )
    declared_arguments.append(
         DeclareLaunchArgument(
            'drive_1_pos',
            default_value='FR_',
            description='Drive module 1 position')
    )
    declared_arguments.append(
         DeclareLaunchArgument(
            'drive_2_pos',
            default_value='FL_',
            description='Drive module 2 position')
    )
    declared_arguments.append(
         DeclareLaunchArgument(
            'drive_3_pos',
            default_value='RR_',
            description='Drive module 3 position')
    )
    declared_arguments.append(
         DeclareLaunchArgument(
            'drive_4_pos',
            default_value='RL_',
            description='Drive module 4 position ')
    )

    # Initialize Arguments
    namespace = LaunchConfiguration('namespace')
    hardware_plugin = LaunchConfiguration('hardware_plugin')
    use_sim_time = LaunchConfiguration('use_sim_time')
    prefix = LaunchConfiguration("prefix")

    # Get URDF via xacro
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution([FindPackageShare("description"), "urdf/robots", "swerveBOT_2.urdf.xacro"]),
            " ",
            "hw_interface_plugin:=", 
            hardware_plugin,
            " ",
            "namespace:=", 
            namespace,
            " ",
            "prefix:=", 
            prefix,
        ]
    )
    robot_description = {"robot_description": robot_description_content}
   

    robot_controllers = PathJoinSubstitution([FindPackageShare("bringup"), "config", "swerveBOT_2_controllers.yaml"])

    # the steering controller libraries by default publish odometry on a separate topic than /tf

    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        namespace=namespace,
        parameters=[robot_description, robot_controllers, {'use_sim_time': use_sim_time}],
        output="both",
     )
    control_node_require = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=control_node,
            on_exit=[
                LogInfo(msg="Listener exited; tearing down entire system."),
                EmitEvent(event=Shutdown())
            ],
        )
    )
    robot_state_pub_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        namespace=namespace,
        output="both",
        parameters=[robot_description,
                    {'use_sim_time': use_sim_time},
                    {'frame_prefix': [namespace, '/']}],
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        namespace=namespace,
        arguments=["joint_state_broadcaster", "--controller-manager", ['/', namespace, "/controller_manager"]],
    )
    swerve_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        namespace=namespace,
        arguments=["swerve_controller", "--controller-manager", ['/', namespace, "/controller_manager"]],
    )
    FR_drive_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        namespace=namespace,
        arguments=["FR_drive_controller", "--controller-manager", ['/', namespace, "/controller_manager"]],
    )
    RL_drive_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        namespace=namespace,
        arguments=["RL_drive_controller", "--controller-manager", ['/', namespace, "/controller_manager"]],
    )

    # Delay start of robot_controller after `joint_state_broadcaster`
    delay_swerve_controller_spawner_after_joint_state_broadcaster_spawner = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[swerve_controller_spawner],
        )
    )
    delay_FR_drive_controller_spawner_after_swerve_controller_spawner = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=swerve_controller_spawner,
            on_exit=[FR_drive_controller_spawner],
        )
    )
    delay_RL_drive_controller_spawner_after_FR_drive_controller_spawner = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=FR_drive_controller_spawner,
            on_exit=[RL_drive_controller_spawner],
        )
    )
    jointstate_aggregator_node = Node(
        package='bringup',
        executable='jointstate_sim_aggregator_2',
        namespace=namespace,
        name='jointstate_sim_aggregator_2',
        output="screen",
    )
    delay_jointstate_aggregator_node_after_RL_drive_controller_spawner = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=RL_drive_controller_spawner,
            on_exit=[jointstate_aggregator_node],
        )
    )
    convertor_node = Node(
        package="bringup",
        executable="jointstate_to_twist_2",
        namespace=namespace,
        name="jointstate_to_twist_2",
        output="screen",
        parameters=[{
            'prefix': LaunchConfiguration('prefix'),
            'wheel_radius': LaunchConfiguration('wheel_radius'),
            'wheel_drive_len': LaunchConfiguration('wheel_drive_len'),
            'drive_1_pos': LaunchConfiguration('drive_1_pos'),
            'drive_2_pos': LaunchConfiguration('drive_2_pos'),
            'drive_3_pos': LaunchConfiguration('drive_3_pos'),
            'drive_4_pos': LaunchConfiguration('drive_4_pos')
            }],
        arguments=[
            LaunchConfiguration('prefix'),
            LaunchConfiguration('wheel_radius'),
            LaunchConfiguration('wheel_drive_len'),
            LaunchConfiguration('drive_1_pos'),
            LaunchConfiguration('drive_2_pos'),
            LaunchConfiguration('drive_3_pos'),
            LaunchConfiguration('drive_4_pos')
        ]
    )
    delay_convertor_node_after_RL_drive_controller_spawner = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=RL_drive_controller_spawner,
            on_exit=[convertor_node],
        )
    )


    nodes = [
        robot_state_pub_node,
        control_node,
        control_node_require,
        joint_state_broadcaster_spawner,
        delay_swerve_controller_spawner_after_joint_state_broadcaster_spawner,
        delay_FR_drive_controller_spawner_after_swerve_controller_spawner,
        delay_RL_drive_controller_spawner_after_FR_drive_controller_spawner,
        delay_jointstate_aggregator_node_after_RL_drive_controller_spawner,
        delay_convertor_node_after_RL_drive_controller_spawner,
      ]

    return LaunchDescription(declared_arguments + nodes)