from launch import LaunchDescription
from launch.actions import ExecuteProcess, DeclareLaunchArgument, RegisterEventHandler
from launch.conditions import IfCondition, UnlessCondition
from launch.event_handlers import OnProcessExit
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution, LaunchConfiguration
from launch_ros.descriptions import ParameterValue
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    # Declare arguments
    declared_arguments = []
    declared_arguments.append(
        DeclareLaunchArgument(
            "gui",
            default_value="true",
            description="Start RViz2 automatically with this launch file.",
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            "remap_odometry_tf",
            default_value="true",
            description="Remap odometry TF from the steering controller to the TF tree.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            'prefix',
            default_value='',
            description='Prefix for the joint name'
        )
    )
    declared_arguments.append(
         DeclareLaunchArgument(
            'wheel_radius',
            default_value= '0.0825',   # Note: default_value must be a string
            description='Drive module wheel radius '
        )
    )
    declared_arguments.append(
         DeclareLaunchArgument(
            'wheel_drive_len',
            default_value= '0.23',   
            description='Drive module wheel separation '
        )
    )
    declared_arguments.append(
         DeclareLaunchArgument(
            'drive_1_pos',
            default_value='FR_',
            description='Drive module 1 position '
        )
    )
    declared_arguments.append(
         DeclareLaunchArgument(
            'drive_2_pos',
            default_value='FL_',
            description='Drive module 2 position '
        )
    )

    # Initialize Arguments
    gui = LaunchConfiguration("gui")
    remap_odometry_tf = LaunchConfiguration("remap_odometry_tf")


    # Get URDF via xacro
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [FindPackageShare("description"), "urdf/robots", "ackermannBOT.urdf.xacro"]
            ),
        ]
    )
    robot_description = {"robot_description": robot_description_content}
   

    robot_controllers = PathJoinSubstitution([FindPackageShare("bringup"), "config", "ackermannBOT_controllers.yaml"])
    rviz_config_file = PathJoinSubstitution([FindPackageShare("description"), "rviz", "amr_view.rviz"])


    # the steering controller libraries by default publish odometry on a separate topic than /tf
    control_node_remapped = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[robot_controllers],
        output="both",
        remappings=[
            ("~/robot_description", "/robot_description"),
            ("/ackermann_steering_controller/tf_odometry", "/tf"),
        ],
        condition=IfCondition(remap_odometry_tf),
    )
    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[robot_controllers],
        output="both",
        remappings=[
            ("~/robot_description", "/robot_description"),
        ],
        condition=UnlessCondition(remap_odometry_tf),
    )
    robot_state_pub_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[robot_description,],
        remappings=[
            ("~/robot_description", "/robot_description"),
        ],  
    )
    joint_state_publisher_gui_node = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui',
    )
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config_file],
        condition=IfCondition(gui),
    )
    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
    )
    ackermann_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["ackermann_steering_controller", "--controller-manager", "/controller_manager"],
    )
    FR_drive_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["FR_drive_controller", "--controller-manager", "/controller_manager"],
    )
    FL_drive_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["FL_drive_controller", "--controller-manager", "/controller_manager"],
    )

    # Delay rviz start after `joint_state_broadcaster`
    delay_rviz_after_joint_state_broadcaster_spawner = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[rviz_node],
        )
    )
    # Delay start of robot_controller after `joint_state_broadcaster`
    delay_ackermann_controller_spawner_after_joint_state_broadcaster_spawner = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[ackermann_controller_spawner],
        )
    )
    delay_drive_controller_spawner_after_ackermann_controller_spawner = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=ackermann_controller_spawner,
            on_exit=[FR_drive_controller_spawner, FL_drive_controller_spawner],
        )
    )

    convertor_node = Node(
        package="bringup",
        executable="jointstate_to_twist_2",
        name="jointstate_to_twist_2",
        output="screen",
        parameters=[{
            'prefix': LaunchConfiguration('prefix'),
            'wheel_radius': LaunchConfiguration('wheel_radius'),
            'wheel_drive_len': LaunchConfiguration('wheel_drive_len'),
            'drive_1_pos': LaunchConfiguration('drive_1_pos'),
            'drive_2_pos': LaunchConfiguration('drive_2_pos')
        }],
        arguments=[
            LaunchConfiguration('prefix'),
            LaunchConfiguration('wheel_radius'),
            LaunchConfiguration('wheel_drive_len'),
            LaunchConfiguration('drive_1_pos'),
            LaunchConfiguration('drive_2_pos')
        ]
    )
    delay_convertor_node_after_FL_drive_controller_spawner = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=FL_drive_controller_spawner,
            on_exit=[convertor_node],
        )
    )

    jointstate_aggregator_node = Node(
        package='bringup',
        executable='jointstate_aggregator_2',
        name='jointstate_aggregator_2',
        output="screen",
    )
    delay_jointstate_aggregator_node_after_FR_drive_controller_spawner = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=FR_drive_controller_spawner,
            on_exit=[jointstate_aggregator_node],
        )
    )
    nodes = [
        control_node,
        control_node_remapped,
        robot_state_pub_node,
        joint_state_broadcaster_spawner,
        delay_rviz_after_joint_state_broadcaster_spawner,
        delay_ackermann_controller_spawner_after_joint_state_broadcaster_spawner,
        delay_drive_controller_spawner_after_ackermann_controller_spawner,
        delay_convertor_node_after_FL_drive_controller_spawner,
        delay_jointstate_aggregator_node_after_FR_drive_controller_spawner
      ]

    return LaunchDescription(declared_arguments + nodes)