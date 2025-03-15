import sys

import numpy as np

import argparse

from isaacsim import SimulationApp

ROBOT_NAME = "wheelbot_ur"
ROBOT_STAGE_PATH = f"/World/{ROBOT_NAME}"
WHEELBOT_STAGE_PATH = f"{ROBOT_STAGE_PATH}/swerveBOT_910"
MANIPULATOR_STAGE_PATH = f"{ROBOT_STAGE_PATH}/ur10e"
LIDAR_front_STAGE_PATH = f"{WHEELBOT_STAGE_PATH}/RPLIDAR_front/RPLIDAR_S2e"
LIDAR_back_STAGE_PATH = f"{WHEELBOT_STAGE_PATH}/RPLIDAR_back/RPLIDAR_S2e"
#CAMERA_front_STAGE_PATH = f"{WHEELBOT_STAGE_PATH}/Realsense_front/RSD455"

WHEELBOT_GRAPH_PATH = f"{WHEELBOT_STAGE_PATH}/Graph"
MANIPULATOR_GRAPH_PATH = f"{MANIPULATOR_STAGE_PATH}/Graph"
LIDARs_GRAPH_PATH = f"{WHEELBOT_GRAPH_PATH}/Lasers"
#IMU_1_GRAPH_PATH = f"{WHEELBOT_GRAPH_PATH}/Imu"
#CAMERA_front_GRAPH_PATH = f"{WHEELBOT_GRAPH_PATH}/Front_Camera"

CONFIG = {"renderer": "RayTracedLighting", "headless": False}

# Set up command line arguments
parser = argparse.ArgumentParser(description="Wheelbot Isaac Sim demo")
parser.add_argument("--world_file", required=True, help="Full path to the world file")
parser.add_argument("--mobile_robot_file", help="Full path to the mobile robot file")
parser.add_argument("--headless", default="False", help="start the simulation app, with GUI open")
parser.add_argument("--renderer", default="RayTracedLighting", choices=["RayTracedLighting", "PathTracing"], help="Renderer to use")
args, unknown = parser.parse_known_args()

# Start the omniverse application
simulation_app = SimulationApp(CONFIG)


import omni
import os
import omni.graph.core as og
import usdrt.Sdf
from isaacsim.core.api import SimulationContext
from isaacsim.core.utils.stage import add_reference_to_stage
from isaacsim.core.utils.extensions import enable_extension
#enable_extension("isaacsim.robot_setup.assembler")
#from isaacsim.robot_setup.assembler import RobotAssembler, AssembledRobot
#from isaacsim.core.prims import SingleArticulation
from pxr import Gf


# enable ROS2 bridge extension
enable_extension("isaacsim.ros2.bridge")



# disable gamepad camera input 
import carb
carb.settings.get_settings().set("persistent/app/omniverse/gamepadCameraControl", False)

NAMESPACE = f"{os.environ.get('ROS_NAMESPACE')}" if 'ROS_NAMESPACE' in os.environ else ''

simulation_app.update()
simulation_context = SimulationContext(stage_units_in_meters=1.0)

# Loading USD Stage
omni.usd.get_context().open_stage(args.world_file)

simulation_app.update()
print("Loading stage...")
from isaacsim.core.utils.stage import is_stage_loading

while is_stage_loading():
    simulation_app.update()
print("Loading Complete")

# Loading the mobile robot USD
add_reference_to_stage(usd_path=args.mobile_robot_file, prim_path=ROBOT_STAGE_PATH)
omni.kit.commands.execute(
    "IsaacSimTeleportPrim",
    prim_path=ROBOT_STAGE_PATH,
    translation=(0, 0, 0),
    rotation=(0, 0, 0, 0),
)
simulation_app.update()
simulation_app.update()

domain_id = 0

# Publish simulation time to ROS2 as a Clock message
try:
   og.Controller.edit(
        {"graph_path": f"{ROBOT_STAGE_PATH}/Graph/ROS_Clock", "evaluator_name": "execution"},
        {
            og.Controller.Keys.CREATE_NODES: [
                ("OnPlaybackTick", "omni.graph.action.OnPlaybackTick"),
                ("ReadSimTime", "isaacsim.core.nodes.IsaacReadSimulationTime"),
                ("PublishClock", "isaacsim.ros2.bridge.ROS2PublishClock"),
                ("ROS2Context", "isaacsim.ros2.bridge.ROS2Context"),
            ],
            og.Controller.Keys.CONNECT: [
                ("OnPlaybackTick.outputs:tick", "PublishClock.inputs:execIn"),
                ("ROS2Context.outputs:context", "PublishClock.inputs:context"),
                ("ReadSimTime.outputs:simulationTime", "PublishClock.inputs:timeStamp"),
            ],
            og.Controller.Keys.SET_VALUES: [
                ("ReadSimTime.inputs:resetOnStop", False),
            ],
        },
    )
except Exception as e:
    print(e)


# Load wheelbot articulations-------------------------------------------------------------------------------------------
try:
	og.Controller.edit(
        {"graph_path": f"{WHEELBOT_GRAPH_PATH}/mobile_action", "evaluator_name": "execution"},
        {
            og.Controller.Keys.CREATE_NODES: [
                ("OnPlaybackTick", "omni.graph.action.OnPlaybackTick"),
                ("Context", "isaacsim.ros2.bridge.ROS2Context"),
                
                ("ConstructArray_FL", "omni.graph.nodes.ConstructArray"),
                ("ArticulationController_FL", "isaacsim.core.nodes.IsaacArticulationController"),
                ("SubscribeJointState_FL", "isaacsim.ros2.bridge.ROS2SubscribeJointState"),

                ("ConstructArray_FR", "omni.graph.nodes.ConstructArray"),
                ("ArticulationController_FR", "isaacsim.core.nodes.IsaacArticulationController"),
                ("SubscribeJointState_FR", "isaacsim.ros2.bridge.ROS2SubscribeJointState"),

                ("ConstructArray_RL", "omni.graph.nodes.ConstructArray"),
                ("ArticulationController_RL", "isaacsim.core.nodes.IsaacArticulationController"),
                ("SubscribeJointState_RL", "isaacsim.ros2.bridge.ROS2SubscribeJointState"),

                ("ConstructArray_RR", "omni.graph.nodes.ConstructArray"),
                ("ArticulationController_RR", "isaacsim.core.nodes.IsaacArticulationController"),
                ("SubscribeJointState_RR", "isaacsim.ros2.bridge.ROS2SubscribeJointState")
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
                ("SubscribeJointState_FL.inputs:nodeNamespace", NAMESPACE),
                ("SubscribeJointState_FL.inputs:topicName", "FL_drive_joint_commands"),

                ("ConstructArray_FR.inputs:arraySize", 2),
                ("ConstructArray_FR.inputs:arrayType", "token[]"),
                ("ConstructArray_FR.inputs:input0", "FR_drive_left_joint"),
                ("ConstructArray_FR.inputs:input1", "FR_drive_right_joint"),
                ("ArticulationController_FR.inputs:robotPath", WHEELBOT_STAGE_PATH),
                ("SubscribeJointState_FR.inputs:nodeNamespace", NAMESPACE),
                ("SubscribeJointState_FR.inputs:topicName", "FR_drive_joint_commands"),

                ("ConstructArray_RL.inputs:arraySize", 2),
                ("ConstructArray_RL.inputs:arrayType", "token[]"),
                ("ConstructArray_RL.inputs:input0", "RL_drive_left_joint"),
                ("ConstructArray_RL.inputs:input1", "RL_drive_right_joint"),
                ("ArticulationController_RL.inputs:robotPath", WHEELBOT_STAGE_PATH),
                ("SubscribeJointState_RL.inputs:nodeNamespace", NAMESPACE),
                ("SubscribeJointState_RL.inputs:topicName", "RL_drive_joint_commands"),

                ("ConstructArray_RR.inputs:arraySize", 2),
                ("ConstructArray_RR.inputs:arrayType", "token[]"),
                ("ConstructArray_RR.inputs:input0", "RR_drive_left_joint"),
                ("ConstructArray_RR.inputs:input1", "RR_drive_right_joint"),
                ("ArticulationController_RR.inputs:robotPath", WHEELBOT_STAGE_PATH),
                ("SubscribeJointState_RR.inputs:nodeNamespace", NAMESPACE),
                ("SubscribeJointState_RR.inputs:topicName", "RR_drive_joint_commands")
            ],

            og.Controller.Keys.CONNECT: [
                ("OnPlaybackTick.outputs:tick", "ArticulationController_FL.inputs:execIn"),
                ("OnPlaybackTick.outputs:tick", "ArticulationController_FR.inputs:execIn"),
                ("OnPlaybackTick.outputs:tick", "ArticulationController_RL.inputs:execIn"),
                ("OnPlaybackTick.outputs:tick", "ArticulationController_RR.inputs:execIn"),

                ("OnPlaybackTick.outputs:tick", "SubscribeJointState_FL.inputs:execIn"),
                ("OnPlaybackTick.outputs:tick", "SubscribeJointState_FR.inputs:execIn"),
                ("OnPlaybackTick.outputs:tick", "SubscribeJointState_RL.inputs:execIn"),
                ("OnPlaybackTick.outputs:tick", "SubscribeJointState_RR.inputs:execIn"),

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

# Load UR articulations-------------------------------------------------------------------------------------------
# try:
# 	og.Controller.edit(
#         {"graph_path": f"{MANIPULATOR_GRAPH_PATH}/ur_action", "evaluator_name": "execution"},
#         {
#             og.Controller.Keys.CREATE_NODES: [
#                 ("OnPlaybackTick", "omni.graph.action.OnPlaybackTick"),
#                 ("Context", "isaacsim.ros2.bridge.ROS2Context"),
#                 ("ConstructArray_UR", "omni.graph.nodes.ConstructArray"),
#                 ("ArticulationController_UR", "isaacsim.core.nodes.IsaacArticulationController"),
#                 ("SubscribeJointState_UR", "isaacsim.ros2.bridge.ROS2SubscribeJointState"),
#             ],
         
#             og.Controller.Keys.SET_VALUES: [
#                 ("ConstructArray_UR.inputs:arraySize", 6),
#                 ("ConstructArray_UR.inputs:arrayType", "token[]"),
#                 ("ConstructArray_UR.inputs:input0", "shoulder_pan_joint"),
#                 ("ConstructArray_UR.inputs:input1", "shoulder_lift_joint"),
#                 ("ConstructArray_UR.inputs:input2", "elbow_joint"),
#                 ("ConstructArray_UR.inputs:input3", "wrist_1_joint"), 
#                 ("ConstructArray_UR.inputs:input4", "wrist_2_joint"),
#                 ("ConstructArray_UR.inputs:input5", "wrist_3_joint"),       
#                 ("ArticulationController_UR.inputs:targetPrim", MANIPULATOR_STAGE_PATH),   
#                 #("ArticulationController_UR.inputs:robotPath", MANIPULATOR_STAGE_PATH),
#                 ("SubscribeJointState_UR.inputs:nodeNamespace", NAMESPACE),
#                 ("SubscribeJointState_UR.inputs:topicName", "ur_joint_commands"),
#             ],

#             og.Controller.Keys.CONNECT: [
#                 ("OnPlaybackTick.outputs:tick", "ArticulationController_UR.inputs:execIn"),
#                 ("OnPlaybackTick.outputs:tick", "SubscribeJointState_UR.inputs:execIn"),
#                 ("Context.outputs:context", "SubscribeJointState_UR.inputs:context"),
                
#                 ("ConstructArray_UR.outputs:array", "ArticulationController_UR.inputs:jointNames"),
               
#                 ("SubscribeJointState_UR.outputs:velocityCommand", "ArticulationController_UR.inputs:velocityCommand"),
#                 ("SubscribeJointState_UR.outputs:positionCommand", "ArticulationController_UR.inputs:positionCommand"),
#                 ("SubscribeJointState_UR.outputs:effortCommand", "ArticulationController_UR.inputs:effortCommand"),
#             ],  
#         },
#     )
# except Exception as e:
#     print(e)

try:
    og.Controller.edit(
        {"graph_path": f"{WHEELBOT_GRAPH_PATH}/Publish_Joints", "evaluator_name": "execution"},
        {
            og.Controller.Keys.CREATE_NODES: [
                ("OnPlaybackTick", "omni.graph.action.OnPlaybackTick"),
                ("Context", "isaacsim.ros2.bridge.ROS2Context"),
                ("ReadSimulationTime", "isaacsim.core.nodes.IsaacReadSimulationTime"),
                ("PublishJointState", "isaacsim.ros2.bridge.ROS2PublishJointState"),
            ],
            
            og.Controller.Keys.SET_VALUES: [
                ("PublishJointState.inputs:nodeNamespace", NAMESPACE), 
                ("PublishJointState.inputs:topicName", "isaac_amr_joint_states"), 
                ("PublishJointState.inputs:targetPrim", WHEELBOT_STAGE_PATH)
            ],
            
            og.Controller.Keys.CONNECT: [
                ("OnPlaybackTick.outputs:tick", "PublishJointState.inputs:execIn"),
                ("ReadSimulationTime.outputs:simulationTime", "PublishJointState.inputs:timeStamp"),
                ("Context.outputs:context", "PublishJointState.inputs:context"),

            ],  
        }  
    )
except Exception as e:
    print(e)


try:
    og.Controller.edit(
        {"graph_path": f"{WHEELBOT_GRAPH_PATH}/Publish_Odom", "evaluator_name": "execution"},
        {
            og.Controller.Keys.CREATE_NODES: [
                ("OnPlaybackTick", "omni.graph.action.OnPlaybackTick"),
                ("ReadSimTime", "isaacsim.core.nodes.IsaacReadSimulationTime"),
                ("Context", "isaacsim.ros2.bridge.ROS2Context"),
                ("ComputeOdometry", "isaacsim.core.nodes.IsaacComputeOdometry"),
                ("PublishOdometry", "isaacsim.ros2.bridge.ROS2PublishOdometry"),
                ("PublishOdomTF", "isaacsim.ros2.bridge.ROS2PublishRawTransformTree")

            ],
            og.Controller.Keys.CONNECT: [
                ("OnPlaybackTick.outputs:tick", "ComputeOdometry.inputs:execIn"),
                ("OnPlaybackTick.outputs:tick", "PublishOdometry.inputs:execIn"),
                ("ComputeOdometry.outputs:linearVelocity", "PublishOdometry.inputs:linearVelocity"),
                ("ComputeOdometry.outputs:angularVelocity", "PublishOdometry.inputs:angularVelocity"),
                ("ComputeOdometry.outputs:position", "PublishOdometry.inputs:position"),
                ("ComputeOdometry.outputs:orientation", "PublishOdometry.inputs:orientation"),
                ("Context.outputs:context", "PublishOdometry.inputs:context"),
                ("ReadSimTime.outputs:simulationTime", "PublishOdometry.inputs:timeStamp"),

                ("OnPlaybackTick.outputs:tick", "PublishOdomTF.inputs:execIn"),
                ("ComputeOdometry.outputs:position", "PublishOdomTF.inputs:translation"),
                ("ComputeOdometry.outputs:orientation", "PublishOdomTF.inputs:rotation"),
                ("Context.outputs:context", "PublishOdomTF.inputs:context"),
                ("ReadSimTime.outputs:simulationTime", "PublishOdomTF.inputs:timeStamp"),

            ],
            og.Controller.Keys.SET_VALUES: [
                ("ComputeOdometry.inputs:chassisPrim", WHEELBOT_STAGE_PATH),
                ("PublishOdometry.inputs:nodeNamespace", NAMESPACE),
                ("PublishOdometry.inputs:topicName", "odom"),
                ("PublishOdometry.inputs:odomFrameId", "odom"),
                ("PublishOdometry.inputs:chassisFrameId", "chassis_link"),

                ("PublishOdomTF.inputs:nodeNamespace", NAMESPACE),
                ("PublishOdomTF.inputs:childFrameId", "base_footprint"),
                ("PublishOdomTF.inputs:parentFrameId", "odom"),
                ("PublishOdomTF.inputs:topicName", "tf"),
            ],
        },
    )
except Exception as e:
    print(e)

# Load front camera Realsense D455-------------------------------------------------------------------------------------------
# try:
# 	og.Controller.edit(
#         {   "graph_path": CAMERA_front_GRAPH_PATH, "evaluator_name": "execution" },
#         {
#             og.Controller.Keys.CREATE_NODES: [
#                 ("OnPlaybackTick", "omni.graph.action.OnPlaybackTick"),
#                 ("ROS2Context", "isaacsim.ros2.bridge.ROS2Context"),
#                 ("RunOneSimulationFrame", "isaacsim.core.nodes.OgnIsaacRunOneSimulationFrame"),

#                 #Color camera
#                 ("color_camera_render_product",  "isaacsim.core.nodes.IsaacCreateRenderProduct"),
#                 ("color_camera_publish_rgb", "isaacsim.ros2.bridge.ROS2CameraHelper"),
#                 ("color_camera_publish_info", "isaacsim.ros2.bridge.ROS2CameraHelper"),

#                 # Pseudo depth-camera
#                 ("depth_camera_render_product",  "isaacsim.core.nodes.IsaacCreateRenderProduct"),
#                 ("depth_camera_publish_depth", "isaacsim.ros2.bridge.ROS2CameraHelper"),
#                 ("depth_camera_publish_info", "isaacsim.ros2.bridge.ROS2CameraHelper"),

#             ],
#             og.Controller.Keys.CONNECT: [
#                 ("OnPlaybackTick.outputs:tick", "RunOneSimulationFrame.inputs:execIn"),

#                 # Color camera
#                 ("RunOneSimulationFrame.outputs:step", "color_camera_render_product.inputs:execIn"),
#                 ("color_camera_render_product.outputs:execOut", "color_camera_publish_info.inputs:execIn"),
#                 ("color_camera_render_product.outputs:renderProductPath", "color_camera_publish_info.inputs:renderProductPath"),
#                 ("ROS2Context.outputs:context", "color_camera_publish_info.inputs:context"),

#                 ("color_camera_render_product.outputs:execOut", "color_camera_publish_rgb.inputs:execIn"),
#                 ("color_camera_render_product.outputs:renderProductPath", "color_camera_publish_rgb.inputs:renderProductPath"),
#                 ("ROS2Context.outputs:context", "color_camera_publish_rgb.inputs:context"),

#                 # Pseudo depth-camera
#                 ("RunOneSimulationFrame.outputs:step", "depth_camera_render_product.inputs:execIn"),
     
#                 ("depth_camera_render_product.outputs:execOut", "depth_camera_publish_info.inputs:execIn"),
#                 ("depth_camera_render_product.outputs:renderProductPath", "depth_camera_publish_info.inputs:renderProductPath"),
#                 ("ROS2Context.outputs:context", "depth_camera_publish_info.inputs:context"),
    
#                 ("depth_camera_render_product.outputs:execOut", "depth_camera_publish_depth.inputs:execIn"),
#                 ("depth_camera_render_product.outputs:renderProductPath", "depth_camera_publish_depth.inputs:renderProductPath"),
#                 ("ROS2Context.outputs:context", "depth_camera_publish_depth.inputs:context"),
#             ],   
#             og.Controller.Keys.SET_VALUES: [
#                 # Color camera 
#                 ("color_camera_render_product.inputs:cameraPrim", f"{CAMERA_front_STAGE_PATH}/Camera_OmniVision_OV9782_Color"),
                
#                 ("color_camera_publish_rgb.inputs:frameId",  f"{NAMESPACE}/front_camera_link"),
#                 ("color_camera_publish_rgb.inputs:nodeNamespace", NAMESPACE),
#                 ("color_camera_publish_rgb.inputs:topicName", "/front_camera/color/image_raw"),
#                 ("color_camera_publish_rgb.inputs:type", "rgb"),

#                 ("color_camera_publish_info.inputs:frameId",  f"{NAMESPACE}/front_camera_link"),
#                 ("color_camera_publish_info.inputs:nodeNamespace", NAMESPACE),
#                 ("color_camera_publish_info.inputs:topicName", "/front_camera/color/info"),
#                 ("color_camera_publish_info.inputs:type", "camera_info"),

#                 # Pseudo depth-camera
#                 ("depth_camera_render_product.inputs:cameraPrim",  f"{CAMERA_front_STAGE_PATH}/Camera_Pseudo_Depth"),
                
#                 ("depth_camera_publish_depth.inputs:frameId", f"{NAMESPACE}/front_camera_link"),
#                 ("depth_camera_publish_depth.inputs:nodeNamespace", NAMESPACE),
#                 ("depth_camera_publish_depth.inputs:topicName", "/front_camera/depth/image_rect_raw"),
#                 ("depth_camera_publish_depth.inputs:type", "depth"),

#                 ("depth_camera_publish_info.inputs:frameId", f"{NAMESPACE}/front_camera_link"),            
#                 ("depth_camera_publish_info.inputs:nodeNamespace", NAMESPACE),
#                 ("depth_camera_publish_info.inputs:topicName", "/front_camera/depth/info"),
#                 ("depth_camera_publish_info.inputs:type", "camera_info"),
#              ],
#         },
#     )
# except Exception as e:
#     print(e)

# Load IMU from front camera Realsense D455-------------------------------------------------------------------------------------------
# try:
# 	og.Controller.edit(
#         { "graph_path": IMU_1_GRAPH_PATH, "evaluator_name": "execution"},
#         {
#             og.Controller.Keys.CREATE_NODES: [    
#                 ("OnPlaybackTick", "omni.graph.action.OnPlaybackTick"),
#                 ("SimulationGate", "isaacsim.core.nodes.IsaacSimulationGate"),
#                 ("ROS2Context", "isaacsim.ros2.bridge.ROS2Context"),
#                 ("ReadSimTime", "isaacsim.core.nodes.IsaacReadSimulationTime"),

#                 ("IsaacReadImu", "isaacsim.sensor.IsaacReadIMU"),
#                 ("PublishImu", "isaacsim.ros2.bridge.ROS2PublishImu"),
#             ],
#             og.Controller.Keys.CONNECT: [
#                 ("OnPlaybackTick.outputs:tick", "SimulationGate.inputs:execIn"),
#                 ("SimulationGate.outputs:execOut", "IsaacReadImu.inputs:execIn"),
#                 ("IsaacReadImu.outputs:execOut", "PublishImu.inputs:execIn"),
#                 ("ROS2Context.outputs:context", "PublishImu.inputs:context"),
#                 ("ReadSimTime.outputs:simulationTime", "PublishImu.inputs:timeStamp"),

#                 ("IsaacReadImu.outputs:angVel", "PublishImu.inputs:angularVelocity"),
#                 ("IsaacReadImu.outputs:linAcc", "PublishImu.inputs:linearAcceleration"),
#                 ("IsaacReadImu.outputs:orientation", "PublishImu.inputs:orientation"),
#             ],
#             og.Controller.Keys.SET_VALUES: [
#                 ("IsaacReadImu.inputs:imuPrim",  f"{LIDAR_front_STAGE_PATH}/Imu_Sensor"),
                
#                 ("PublishImu.inputs:frameId", f"{NAMESPACE}/front_camera"),
#                 ("PublishImu.inputs:nodeNamespace", NAMESPACE),
#                 ("PublishImu.inputs:topicName", "/imu"),

#             ],
#         }
#     )
# except Exception as e:
#     print(e)

# Load LIDARs-------------------------------------------------------------------------------------------
try:
	og.Controller.edit(
        {   "graph_path": f"{LIDARs_GRAPH_PATH}", "evaluator_name": "execution" },
        {
            og.Controller.Keys.CREATE_NODES: [
                ("OnPlaybackTick", "omni.graph.action.OnPlaybackTick"),
                ("ROS2Context", "isaacsim.ros2.bridge.ROS2Context"),
                ("RunOneSimulationFrame", "isaacsim.core.nodes.OgnIsaacRunOneSimulationFrame"),

                ("front_lidar_render_product",  "isaacsim.core.nodes.IsaacCreateRenderProduct"),
                ("front_lidar_publish_laser_scan", "isaacsim.ros2.bridge.ROS2RtxLidarHelper"),

                ("back_lidar_render_product",  "isaacsim.core.nodes.IsaacCreateRenderProduct"),
                ("back_lidar_publish_laser_scan", "isaacsim.ros2.bridge.ROS2RtxLidarHelper"),
     
            ],
            og.Controller.Keys.CONNECT: [
                ("OnPlaybackTick.outputs:tick", "RunOneSimulationFrame.inputs:execIn"),
                ("RunOneSimulationFrame.outputs:step", "front_lidar_render_product.inputs:execIn"),
                ("RunOneSimulationFrame.outputs:step", "back_lidar_render_product.inputs:execIn"),

                ("front_lidar_render_product.outputs:execOut", "front_lidar_publish_laser_scan.inputs:execIn"),
                ("front_lidar_render_product.outputs:renderProductPath", "front_lidar_publish_laser_scan.inputs:renderProductPath"),
                ("ROS2Context.outputs:context", "front_lidar_publish_laser_scan.inputs:context"),
                
                ("back_lidar_render_product.outputs:execOut", "back_lidar_publish_laser_scan.inputs:execIn"),
                ("back_lidar_render_product.outputs:renderProductPath", "back_lidar_publish_laser_scan.inputs:renderProductPath"),
                ("ROS2Context.outputs:context", "back_lidar_publish_laser_scan.inputs:context"),

            ],   
            og.Controller.Keys.SET_VALUES: [
                ("front_lidar_render_product.inputs:cameraPrim", f"{LIDAR_front_STAGE_PATH}/RPLIDAR_S2E"),
                
                ("front_lidar_publish_laser_scan.inputs:frameId",  "front_lidar_laser"),
                ("front_lidar_publish_laser_scan.inputs:nodeNamespace", NAMESPACE),
                ("front_lidar_publish_laser_scan.inputs:topicName", "front_lidar/scan"),
                ("front_lidar_publish_laser_scan.inputs:type", "laser_scan"),

                ("back_lidar_render_product.inputs:cameraPrim", f"{LIDAR_back_STAGE_PATH}/RPLIDAR_S2E"),
                
                ("back_lidar_publish_laser_scan.inputs:frameId",  "back_lidar_laser"),
                ("back_lidar_publish_laser_scan.inputs:nodeNamespace", NAMESPACE),
                ("back_lidar_publish_laser_scan.inputs:topicName", "back_lidar/scan"),
                ("back_lidar_publish_laser_scan.inputs:type", "laser_scan"),

             ],
        },
    )
except Exception as e:
    print(e)

simulation_app.update()

# need to initialize physics getting any articulation..etc
simulation_context.initialize_physics()

simulation_app.update()


simulation_context.play()

while simulation_app.is_running():
    # Run with a fixed step size
    simulation_context.step(render=True)   

simulation_context.stop()
simulation_app.close()
