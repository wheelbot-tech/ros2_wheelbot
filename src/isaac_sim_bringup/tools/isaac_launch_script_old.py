import sys

import numpy as np

import argparse

from isaacsim import SimulationApp



WHEELBOT_STAGE_PATH = "/wheelbot"
BASE_LINK_STAGE_PATH = "/wheelbot/base_link"
BACKGROUND_STAGE_PATH = "/background"
IMU_PATH = "/wheelbot/chassis_link/visuals/Imu_Sensor"

CONFIG = {"renderer": "RayTracedLighting", "headless": False}

# Set up command line arguments
parser = argparse.ArgumentParser(description="Wheelbot Isaac sim demo")
parser.add_argument("--world_file", required=True, help="Full path to the world file")
parser.add_argument("--robot_file", help="Full path to the robot file")
parser.add_argument("--headless", default="False", help="Run stage headless")
parser.add_argument("--renderer", default="RayTracedLighting", choices=["RayTracedLighting", "PathTracing"], help="Renderer to use")
args, unknown = parser.parse_known_args()

# Start the omniverse application
simulation_app = SimulationApp(CONFIG)


import omni
import omni.graph.core as og
import usdrt.Sdf
from omni.isaac.core import SimulationContext
from omni.isaac.dynamic_control import _dynamic_control
from omni.isaac.core.utils.extensions import enable_extension
from omni.isaac.core.utils import extensions, prims, rotations, stage, viewports
from pxr import Gf

# enable ROS2 bridge extension
enable_extension("omni.isaac.ros2_bridge")

# disable gamepad camera input 
import carb
carb.settings.get_settings().set("persistent/app/omniverse/gamepadCameraControl", False)

simulation_app.update()

simulation_context = SimulationContext(stage_units_in_meters=1.0)
dc=_dynamic_control.acquire_dynamic_control_interface()

# Preparing stage
viewports.set_camera_view(eye=np.array([1.2, 1.2, 0.8]), target=np.array([0, 0, 0.5]))
# Loading the environment
stage.add_reference_to_stage(args.world_file, BACKGROUND_STAGE_PATH)


# Loading the wheelbot robot USD
prims.create_prim(
    WHEELBOT_STAGE_PATH,
    "Xform",
    position=np.array([0, 0, 0]),
    orientation=rotations.gf_rotation_to_np_array(Gf.Rotation(Gf.Vec3d(0, 0, 0), 0)),
    usd_path=args.robot_file,
)

simulation_app.update()

# Load articulations-------------------------------------------------------------------------------------------
domain_id = 0

try:
	og.Controller.edit(
        {"graph_path": "/ActJoints_Graph", "evaluator_name": "execution"},
        {
            og.Controller.Keys.CREATE_NODES: [
                ("OnImpulseEvent", "omni.graph.action.OnImpulseEvent"),
                ("Context", "omni.isaac.ros2_bridge.ROS2Context"),
                
                ("ConstructArray_FL", "omni.graph.nodes.ConstructArray"),
                ("ArticulationController_FL", "omni.isaac.core_nodes.IsaacArticulationController"),
                ("SubscribeJointState_FL", "omni.isaac.ros2_bridge.ROS2SubscribeJointState"),

                ("ConstructArray_FR", "omni.graph.nodes.ConstructArray"),
                ("ArticulationController_FR", "omni.isaac.core_nodes.IsaacArticulationController"),
                ("SubscribeJointState_FR", "omni.isaac.ros2_bridge.ROS2SubscribeJointState"),

                ("ConstructArray_RL", "omni.graph.nodes.ConstructArray"),
                ("ArticulationController_RL", "omni.isaac.core_nodes.IsaacArticulationController"),
                ("SubscribeJointState_RL", "omni.isaac.ros2_bridge.ROS2SubscribeJointState"),

                ("ConstructArray_RR", "omni.graph.nodes.ConstructArray"),
                ("ArticulationController_RR", "omni.isaac.core_nodes.IsaacArticulationController"),
                ("SubscribeJointState_RR", "omni.isaac.ros2_bridge.ROS2SubscribeJointState")
            ],
            og.Controller.Keys.CREATE_ATTRIBUTES: [
                ("ConstructArray_FL.inputs:input1", "token"),
                ("ConstructArray_FL.inputs:input2", "token"),

                ("ConstructArray_FR.inputs:input1", "token"),
                ("ConstructArray_FR.inputs:input2", "token"),

                ("ConstructArray_RL.inputs:input1", "token"),
                ("ConstructArray_RL.inputs:input2", "token"),

                ("ConstructArray_RR.inputs:input1", "token"),
                ("ConstructArray_RR.inputs:input2", "token")
            ],            
            og.Controller.Keys.SET_VALUES: [
                ("ConstructArray_FL.inputs:arraySize", 2),
                ("ConstructArray_FL.inputs:arrayType", "token[]"),
                ("ConstructArray_FL.inputs:input0", "FL_drive_left_joint"),
                ("ConstructArray_FL.inputs:input1", "FL_drive_right_joint"),
                ("ArticulationController_FL.inputs:robotPath", WHEELBOT_STAGE_PATH),
                ("SubscribeJointState_FL.inputs:topicName", "/FL_drive_joint_commands"),

                ("ConstructArray_FR.inputs:arraySize", 2),
                ("ConstructArray_FR.inputs:arrayType", "token[]"),
                ("ConstructArray_FR.inputs:input0", "FR_drive_left_joint"),
                ("ConstructArray_FR.inputs:input1", "FR_drive_right_joint"),
                ("ArticulationController_FR.inputs:robotPath", WHEELBOT_STAGE_PATH),
                ("SubscribeJointState_FR.inputs:topicName", "/FR_drive_joint_commands"),

                ("ConstructArray_RL.inputs:arraySize", 2),
                ("ConstructArray_RL.inputs:arrayType", "token[]"),
                ("ConstructArray_RL.inputs:input0", "RL_drive_left_joint"),
                ("ConstructArray_RL.inputs:input1", "RL_drive_right_joint"),
                ("ArticulationController_RL.inputs:robotPath", WHEELBOT_STAGE_PATH),
                ("SubscribeJointState_RL.inputs:topicName", "/RL_drive_joint_commands"),

                ("ConstructArray_RR.inputs:arraySize", 2),
                ("ConstructArray_RR.inputs:arrayType", "token[]"),
                ("ConstructArray_RR.inputs:input0", "RR_drive_left_joint"),
                ("ConstructArray_RR.inputs:input1", "RR_drive_right_joint"),
                ("ArticulationController_RR.inputs:robotPath", WHEELBOT_STAGE_PATH),
                ("SubscribeJointState_RR.inputs:topicName", "/RR_drive_joint_commands")
            ],

            og.Controller.Keys.CONNECT: [
                ("OnImpulseEvent.outputs:execOut", "ArticulationController_FL.inputs:execIn"),
                ("OnImpulseEvent.outputs:execOut", "ArticulationController_FR.inputs:execIn"),
                ("OnImpulseEvent.outputs:execOut", "ArticulationController_RL.inputs:execIn"),
                ("OnImpulseEvent.outputs:execOut", "ArticulationController_RR.inputs:execIn"),

                ("OnImpulseEvent.outputs:execOut", "SubscribeJointState_FL.inputs:execIn"),
                ("OnImpulseEvent.outputs:execOut", "SubscribeJointState_FR.inputs:execIn"),
                ("OnImpulseEvent.outputs:execOut", "SubscribeJointState_RL.inputs:execIn"),
                ("OnImpulseEvent.outputs:execOut", "SubscribeJointState_RR.inputs:execIn"),

                ("Context.outputs:context", "SubscribeJointState_FL.inputs:context"),
                ("Context.outputs:context", "SubscribeJointState_FR.inputs:context"),
                ("Context.outputs:context", "SubscribeJointState_RL.inputs:context"),
                ("Context.outputs:context", "SubscribeJointState_RR.inputs:context"),
                
                ("ConstructArray_FL.outputs:array", "ArticulationController_FL.inputs:jointNames"),
                ("ConstructArray_FR.outputs:array", "ArticulationController_FR.inputs:jointNames"),
                ("ConstructArray_RL.outputs:array", "ArticulationController_RL.inputs:jointNames"),
                ("ConstructArray_RR.outputs:array", "ArticulationController_RR.inputs:jointNames"),
                
                ("SubscribeJointState_FL.outputs:velocityCommand", "ArticulationController_FL.inputs:velocityCommand"),
                ("SubscribeJointState_FR.outputs:velocityCommand", "ArticulationController_FR.inputs:velocityCommand"),
                ("SubscribeJointState_RL.outputs:velocityCommand", "ArticulationController_RL.inputs:velocityCommand"),
                ("SubscribeJointState_RR.outputs:velocityCommand", "ArticulationController_RR.inputs:velocityCommand")
            ],  
        },
    )
except Exception as e:
    print(e)

try:
    og.Controller.edit(
        {"graph_path": "/PubJoints_Graph", "evaluator_name": "execution"},
        {
            og.Controller.Keys.CREATE_NODES: [
                ("OnImpulseEvent", "omni.graph.action.OnImpulseEvent"),
                ("Context", "omni.isaac.ros2_bridge.ROS2Context"),
                ("ReadSimulationTime", "omni.isaac.core_nodes.IsaacReadSimulationTime"),
                ("PublishJointState", "omni.isaac.ros2_bridge.ROS2PublishJointState"),
                ("PublishClock", "omni.isaac.ros2_bridge.ROS2PublishClock")
            ],
            
            og.Controller.Keys.SET_VALUES: [
                ("PublishJointState.inputs:topicName", "/isaac_joint_states"), 
                ("PublishJointState.inputs:targetPrim", [usdrt.Sdf.Path(WHEELBOT_STAGE_PATH)])
            ],
            
            og.Controller.Keys.CONNECT: [
                ("OnImpulseEvent.outputs:execOut", "PublishJointState.inputs:execIn"),
                ("OnImpulseEvent.outputs:execOut", "PublishClock.inputs:execIn"),
                ("ReadSimulationTime.outputs:simulationTime", "PublishJointState.inputs:timeStamp"),
                ("ReadSimulationTime.outputs:simulationTime", "PublishClock.inputs:timeStamp"),
                ("Context.outputs:context", "PublishJointState.inputs:context"),
                ("Context.outputs:context", "PublishClock.inputs:context")
            ],  
        }  
    )
except Exception as e:
    print(e)

try:
    og.Controller.edit(
        {"graph_path": "/Imu_Graph", "evaluator_name": "execution"},
        {
            og.Controller.Keys.CREATE_NODES: [
                ("OnImpulseEvent", "omni.graph.action.OnImpulseEvent"),
                ("SimTime", "omni.isaac.core_nodes.IsaacReadSimulationTime"),
                ("Context", "omni.isaac.ros2_bridge.ROS2Context"),
                ("SimulationGate", "omni.isaac.core_nodes.IsaacSimulationGate"),
                ("IsaacReadImu", "omni.isaac.sensor.IsaacReadIMU"),
                ("PublishImu", "omni.isaac.ros2_bridge.ROS2PublishImu")
            ],
            og.Controller.Keys.SET_VALUES: [
                ("SimulationGate.inputs:step", 4),
                ("IsaacReadImu.inputs:imuPrim", "/wheelbot/Realsense/RSD455/Imu_Sensor"),
                ("PublishImu.inputs:topicName", "/realsense_imu")
            ],
            og.Controller.Keys.CONNECT: [                  
                ("OnImpulseEvent.outputs:execOut", "SimulationGate.inputs:execIn"),
                ("SimulationGate.outputs:execOut", "IsaacReadImu.inputs:execIn"),
                ("IsaacReadImu.outputs:execOut", "PublishImu.inputs:execIn"),
                ("Context.outputs:context", "PublishImu.inputs:context"),
                ("SimTime.outputs:simulationTime", "PublishImu.inputs:timeStamp"),

                ("IsaacReadImu.outputs:angVel", "PublishImu.inputs:angularVelocity"),
                ("IsaacReadImu.outputs:linAcc", "PublishImu.inputs:linearAcceleration"),
                ("IsaacReadImu.outputs:orientation", "PublishImu.inputs:orientation")
            ],
        }
    )
except Exception as e:
    print(e)

try:
    og.Controller.edit(
        {"graph_path": "/TF_Graph", "evaluator_name": "execution"},
        {
            og.Controller.Keys.CREATE_NODES: [
                ("OnImpulseEvent", "omni.graph.action.OnImpulseEvent"),
                ("ReadSimTime", "omni.isaac.core_nodes.IsaacReadSimulationTime"),
                ("Context", "omni.isaac.ros2_bridge.ROS2Context"),
                ("PublishTF", "omni.isaac.ros2_bridge.ROS2PublishTransformTree")

            ],
        
            og.Controller.Keys.SET_VALUES: [
                ("PublishTF.inputs:parentPrim", "/wheelbot/base_link"),
                ("PublishTF.inputs:targetPrims", WHEELBOT_STAGE_PATH),
            ],

            og.Controller.Keys.CONNECT: [
                ("OnImpulseEvent.outputs:execOut", "PublishTF.inputs:execIn"),
                ("Context.outputs:context", "PublishTF.inputs:context"),
                ("ReadSimTime.outputs:simulationTime", "PublishTF.inputs:timeStamp"),
            ]
        },
    )
except Exception as e:
    print(e)


try:
    og.Controller.edit(
        {"graph_path": "/Odom_Graph", "evaluator_name": "execution"},
        {
            og.Controller.Keys.CREATE_NODES: [
                ("OnImpulseEvent", "omni.graph.action.OnImpulseEvent"),
                ("ReadSimTime", "omni.isaac.core_nodes.IsaacReadSimulationTime"),
                ("Context", "omni.isaac.ros2_bridge.ROS2Context"),
                ("ComputeOdometry", "omni.isaac.core_nodes.IsaacComputeOdometry"),
                ("PublishOdometry", "omni.isaac.ros2_bridge.ROS2PublishOdometry"),
                ("PublishOdomTF", "omni.isaac.ros2_bridge.ROS2PublishRawTransformTree"),
                #("PublishBaseTF", "omni.isaac.ros2_bridge.ROS2PublishRawTransformTree")

            ],
            og.Controller.Keys.CONNECT: [
                ("OnImpulseEvent.outputs:execOut", "ComputeOdometry.inputs:execIn"),
                ("OnImpulseEvent.outputs:execOut", "PublishOdometry.inputs:execIn"),
                ("ComputeOdometry.outputs:linearVelocity", "PublishOdometry.inputs:linearVelocity"),
                ("ComputeOdometry.outputs:angularVelocity", "PublishOdometry.inputs:angularVelocity"),
                ("ComputeOdometry.outputs:position", "PublishOdometry.inputs:position"),
                ("ComputeOdometry.outputs:orientation", "PublishOdometry.inputs:orientation"),
                ("Context.outputs:context", "PublishOdometry.inputs:context"),
                ("ReadSimTime.outputs:simulationTime", "PublishOdometry.inputs:timeStamp"),

                ("OnImpulseEvent.outputs:execOut", "PublishOdomTF.inputs:execIn"),
                ("ComputeOdometry.outputs:position", "PublishOdomTF.inputs:translation"),
                ("ComputeOdometry.outputs:orientation", "PublishOdomTF.inputs:rotation"),
                ("Context.outputs:context", "PublishOdomTF.inputs:context"),
                ("ReadSimTime.outputs:simulationTime", "PublishOdomTF.inputs:timeStamp"),

                #("OnImpulseEvent.outputs:execOut", "PublishBaseTF.inputs:execIn"),
                #("Context.outputs:context", "PublishBaseTF.inputs:context"),
                #("ReadSimTime.outputs:simulationTime", "PublishBaseTF.inputs:timeStamp")

            ],
            og.Controller.Keys.SET_VALUES: [
                ("ComputeOdometry.inputs:chassisPrim", "/wheelbot"),
                ("PublishOdometry.inputs:topicName", "/odom"),
                ("PublishOdometry.inputs:odomFrameId", "odom"),
                ("PublishOdometry.inputs:chassisFrameId", "chassis_link"),

                ("PublishOdomTF.inputs:childFrameId", "base_link"),
                ("PublishOdomTF.inputs:parentFrameId", "odom"),

                #("PublishBaseTF.inputs:childFrameId", "odom"),
                #("PublishBaseTF.inputs:parentFrameId", "world")
            ],
        },
    )
except Exception as e:
    print(e)


simulation_app.update()
simulation_app.update()

def follow_robot():
    # Obține transformarea robotului
    object=dc.get_rigid_body(WHEELBOT_STAGE_PATH)
    object_pose=dc.get_rigid_body_pose(object)

    if object:
        robot_position = object_pose.p  # Coordonatele (x, y, z)
        robot_orientation = object_pose.r  # Quaternionul
        
        # Setează poziția camerei față de robot
        eye = np.array([robot_position.x + 1.2, robot_position.y + 1.2, robot_position.z + 0.8])
        target = np.array([robot_position.x, robot_position.y, robot_position.z + 0.5])
        
        # Actualizează camera
        viewports.set_camera_view(eye=eye, target=target)

# need to initialize physics getting any articulation..etc
simulation_context.initialize_physics()

simulation_context.play()

while simulation_app.is_running():
    #follow_robot()
    # Run with a fixed step size
    simulation_context.step(render=True)
    # Tick the Publish/Subscribe JointState and Publish Clock nodes each frame
    og.Controller.set(og.Controller.attribute("/ActJoints_Graph/OnImpulseEvent.state:enableImpulse"), True)
    og.Controller.set(og.Controller.attribute("/PubJoints_Graph/OnImpulseEvent.state:enableImpulse"), True)
    #og.Controller.set(og.Controller.attribute("/Imu_Graph/OnImpulseEvent.state:enableImpulse"), True)
    #og.Controller.set(og.Controller.attribute("/TF_Graph/OnImpulseEvent.state:enableImpulse"), True)
    og.Controller.set(og.Controller.attribute("/Odom_Graph/OnImpulseEvent.state:enableImpulse"), True)

simulation_context.stop()
simulation_app.close()
