from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler, LogInfo, EmitEvent
from launch.conditions import IfCondition, UnlessCondition
from launch.event_handlers import OnProcessExit
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.events import Shutdown
    

def generate_launch_description():

    # Declare arguments
    declared_arguments = []
    declared_arguments.append(
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='True',
            description='Use sim time if true')
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            'use_stamped',
            default_value='False',
            description='Use TwistStamped cmd_vel')
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            'namespace',
            default_value='',  # 'robot1'
            description='The namespace of nodes and links')
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            'hardware_plugin',
            default_value='topic_based_ros2_control/TopicBasedSystem',
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
            default_value= '0.230',   
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
    use_stamped = LaunchConfiguration('use_stamped')
    prefix = LaunchConfiguration("prefix")

    # Get URDF via xacro
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution([FindPackageShare("description_mobile"), "urdf/robots", "swerveBOT_910.urdf.xacro"]),
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

    bringup_path = FindPackageShare("bringup_mobile")
    robot_controllers = PathJoinSubstitution([bringup_path, "config", "swerveBOT_controllers.yaml"]) 
    
    scan_filter_config = PathJoinSubstitution([bringup_path, "config", "angular_filter_scan.yaml"])
    # the steering controller libraries by default publish odometry on a separate topic than /tf

    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        #namespace=namespace,
        parameters=[robot_description, robot_controllers, {'use_sim_time': use_sim_time}],
        output="both",
        condition=IfCondition(use_stamped)
     )

    # control_node_require = RegisterEventHandler(
    #     event_handler=OnProcessExit(
    #         target_action=control_node,
    #         on_exit=[
    #             LogInfo(msg="Listener exited; tearing down entire system."),
    #             EmitEvent(event=Shutdown())
    #         ],
    #     )
    # )
    robot_state_pub_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        #namespace=namespace, 
        output="both",
        parameters=[robot_description,
                    {'use_sim_time': use_sim_time},
                    #{'frame_prefix': [namespace, '/']}
                    ],
        remappings = [('/tf', 'tf'), ('/tf_static', 'tf_static')]
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        #namespace=namespace,
        #arguments=["joint_state_broadcaster", "--controller-manager", ["/", namespace, "/controller_manager"]],
        arguments=["joint_state_broadcaster"],
    )
    swerve_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        #namespace=namespace,
        #arguments=["swerve_controller", "--controller-manager", ["/", namespace, "/controller_manager"]],
        arguments=["swerve_controller"],
    )
    FR_drive_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        #namespace=namespace,
        #arguments=["FR_drive_controller", "--controller-manager", ["/", namespace, "/controller_manager"]],
        arguments=["FR_drive_controller"],
    )
    FL_drive_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        #namespace=namespace,
        #arguments=["FL_drive_controller", "--controller-manager", ["/", namespace, "/controller_manager"]],
        arguments=["FL_drive_controller"],
    )
    RR_drive_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        #namespace=namespace,
        #arguments=["RR_drive_controller", "--controller-manager", ["/", namespace, "/controller_manager"]],
        arguments=["RR_drive_controller"],
    )
    RL_drive_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        #namespace=namespace,
        #arguments=["RL_drive_controller", "--controller-manager", ["/", namespace, "/controller_manager"]],
        arguments=["RL_drive_controller"],
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
    delay_FL_drive_controller_spawner_after_FR_drive_controller_spawner = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=FR_drive_controller_spawner,
            on_exit=[FL_drive_controller_spawner],
        )
    )
    delay_RR_drive_controller_spawner_after_FL_drive_controller_spawner = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=FL_drive_controller_spawner,
            on_exit=[RR_drive_controller_spawner],
        )
    )
    delay_RL_drive_controller_spawner_after_RR_drive_controller_spawner = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=RR_drive_controller_spawner,
            on_exit=[RL_drive_controller_spawner],
        )
    )
    jointstate_aggregator_node = Node(
        package='bringup_mobile',
        executable='jointstate_aggregator_4',
        namespace=namespace,
        name='jointstate_aggregator_4',
        output="screen",
    )
    delay_jointstate_aggregator_node_after_spawners = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=RL_drive_controller_spawner,
            on_exit=[jointstate_aggregator_node],
        )
    )
    convertor_node = Node(
        package="bringup_mobile",
        executable="jointstate_to_twist_4",
        namespace=namespace,
        name="jointstate_to_twist_4",
        output="screen",
        parameters=[{
            'prefix': LaunchConfiguration('prefix'),
            'use_stamped': LaunchConfiguration('use_stamped'),
            'wheel_radius': LaunchConfiguration('wheel_radius'),
            'wheel_drive_len': LaunchConfiguration('wheel_drive_len'),
            'drive_1_pos': LaunchConfiguration('drive_1_pos'),
            'drive_2_pos': LaunchConfiguration('drive_2_pos'),
            'drive_3_pos': LaunchConfiguration('drive_3_pos'),
            'drive_4_pos': LaunchConfiguration('drive_4_pos')
        }],
        arguments=[
            LaunchConfiguration('prefix'),
            LaunchConfiguration('use_stamped'),
            LaunchConfiguration('wheel_radius'),
            LaunchConfiguration('wheel_drive_len'),
            LaunchConfiguration('drive_1_pos'),
            LaunchConfiguration('drive_2_pos'),
            LaunchConfiguration('drive_3_pos'),
            LaunchConfiguration('drive_4_pos')
        ]
    )
    delay_convertor_node_after_spawners = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=RL_drive_controller_spawner,
            on_exit=[convertor_node],
        )
    )
    front_laser_filter_node =    Node(
            package="laser_filters",
            executable="scan_to_scan_filter_chain",
            namespace="front_lidar",
            parameters=[scan_filter_config, {'use_sim_time': use_sim_time}],
    )
    delay_front_laser_filter_node_after_spawners = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=RL_drive_controller_spawner,
            on_exit=[front_laser_filter_node],
        )
    )
    back_laser_filter_node =    Node(
            package="laser_filters",
            executable="scan_to_scan_filter_chain",
            namespace="back_lidar",
            parameters=[scan_filter_config, {'use_sim_time': use_sim_time}],
    )
    delay_back_laser_filter_node_after_spawners = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=RL_drive_controller_spawner,
            on_exit=[back_laser_filter_node],
        )
    )

    nodes = [
        robot_state_pub_node,
        control_node,
        joint_state_broadcaster_spawner,
        delay_swerve_controller_spawner_after_joint_state_broadcaster_spawner,
        delay_FR_drive_controller_spawner_after_swerve_controller_spawner,
        delay_FL_drive_controller_spawner_after_FR_drive_controller_spawner,
        delay_RR_drive_controller_spawner_after_FL_drive_controller_spawner,
        delay_RL_drive_controller_spawner_after_RR_drive_controller_spawner,
        delay_jointstate_aggregator_node_after_spawners,
        delay_convertor_node_after_spawners,
        delay_front_laser_filter_node_after_spawners,
        delay_back_laser_filter_node_after_spawners,
      ]

    return LaunchDescription(declared_arguments + nodes)
