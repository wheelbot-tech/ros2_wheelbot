import sys

import numpy as np

import argparse

from isaacsim import SimulationApp


BACKGROUND_STAGE_PATH = "/background"

WHEELBOT_STAGE_PATH = "/wheelbot"

CAMERA_1_STAGE_PATH = "/wheelbot/Realsense_front/RSD455"
CAMERA_1_GRAPH_PATH = "/Graph/Front_Camera"
IMU_1_GRAPH_PATH = "/Graph/Imu"
LIDAR_1_STAGE_PATH = "/wheelbot/RPLIDAR_front/RPLIDAR_S2e"
LIDAR_1_GRAPH_PATH = "/Graph/Front_Laser"


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
import os
import omni.graph.core as og
import omni.kit.viewport.utility
import usdrt.Sdf
from omni.isaac.core import SimulationContext
from omni.isaac.dynamic_control import _dynamic_control
from omni.isaac.core.utils.extensions import enable_extension
from omni.isaac.core.utils import extensions, prims, rotations, stage, viewports
from pxr import Gf, OmniGraphSchema, UsdGeom


# enable ROS2 bridge extension
enable_extension("omni.isaac.ros2_bridge")

# disable gamepad camera input 
import carb
carb.settings.get_settings().set("persistent/app/omniverse/gamepadCameraControl", False)

NAMESPACE = f"{os.environ.get('ROS_NAMESPACE')}" if 'ROS_NAMESPACE' in os.environ else 'robot1'

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
domain_id = 0

# Publish simulation time to ROS2 as a Clock message
try:
   og.Controller.edit(
        {"graph_path": "/Graph/Ros2_Clock", "evaluator_name": "execution"},
        {
            og.Controller.Keys.CREATE_NODES: [
                ("OnPlaybackTick", "omni.graph.action.OnPlaybackTick"),
                ("ReadSimTime", "omni.isaac.core_nodes.IsaacReadSimulationTime"),
                ("PublishClock", "omni.isaac.ros2_bridge.ROS2PublishClock"),
                ("ROS2Context", "omni.isaac.ros2_bridge.ROS2Context"),
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

# Load articulations-------------------------------------------------------------------------------------------
try:
	og.Controller.edit(
        {"graph_path": "/Graph/Action_Joints", "evaluator_name": "execution"},
        {
            og.Controller.Keys.CREATE_NODES: [
                ("OnImpulseEvent", "omni.graph.action.OnImpulseEvent"),
                ("Context", "omni.isaac.ros2_bridge.ROS2Context"),
                
                ("ConstructArray_FR", "omni.graph.nodes.ConstructArray"),
                ("ArticulationController_FR", "omni.isaac.core_nodes.IsaacArticulationController"),
                ("SubscribeJointState_FR", "omni.isaac.ros2_bridge.ROS2SubscribeJointState"),

                ("ConstructArray_RL", "omni.graph.nodes.ConstructArray"),
                ("ArticulationController_RL", "omni.isaac.core_nodes.IsaacArticulationController"),
                ("SubscribeJointState_RL", "omni.isaac.ros2_bridge.ROS2SubscribeJointState"),
            ],
            og.Controller.Keys.CREATE_ATTRIBUTES: [

                ("ConstructArray_FR.inputs:input1", "token"),
                ("ConstructArray_FR.inputs:input2", "token"),

                ("ConstructArray_RL.inputs:input1", "token"),
                ("ConstructArray_RL.inputs:input2", "token"),
            ],            
            og.Controller.Keys.SET_VALUES: [
                ("ConstructArray_FR.inputs:arraySize", 2),
                ("ConstructArray_FR.inputs:arrayType", "token[]"),
                ("ConstructArray_FR.inputs:input0", "FR_drive_left_joint"),
                ("ConstructArray_FR.inputs:input1", "FR_drive_right_joint"),
                ("ArticulationController_FR.inputs:robotPath", WHEELBOT_STAGE_PATH),
                ("SubscribeJointState_FR.inputs:nodeNamespace", f"/{NAMESPACE}"),
                ("SubscribeJointState_FR.inputs:topicName", "FR_drive_joint_commands"),

                ("ConstructArray_RL.inputs:arraySize", 2),
                ("ConstructArray_RL.inputs:arrayType", "token[]"),
                ("ConstructArray_RL.inputs:input0", "RL_drive_left_joint"),
                ("ConstructArray_RL.inputs:input1", "RL_drive_right_joint"),
                ("ArticulationController_RL.inputs:robotPath", WHEELBOT_STAGE_PATH),
                ("SubscribeJointState_RL.inputs:nodeNamespace", f"/{NAMESPACE}"),
                ("SubscribeJointState_RL.inputs:topicName", "RL_drive_joint_commands"),

            ],
            og.Controller.Keys.CONNECT: [
                ("OnImpulseEvent.outputs:execOut", "ArticulationController_FR.inputs:execIn"),
                ("OnImpulseEvent.outputs:execOut", "ArticulationController_RL.inputs:execIn"),

                ("OnImpulseEvent.outputs:execOut", "SubscribeJointState_FR.inputs:execIn"),
                ("OnImpulseEvent.outputs:execOut", "SubscribeJointState_RL.inputs:execIn"),

                ("Context.outputs:context", "SubscribeJointState_FR.inputs:context"),
                ("Context.outputs:context", "SubscribeJointState_RL.inputs:context"),
                
                ("ConstructArray_FR.outputs:array", "ArticulationController_FR.inputs:jointNames"),
                ("ConstructArray_RL.outputs:array", "ArticulationController_RL.inputs:jointNames"),
                
                ("SubscribeJointState_FR.outputs:velocityCommand", "ArticulationController_FR.inputs:velocityCommand"),
                ("SubscribeJointState_RL.outputs:velocityCommand", "ArticulationController_RL.inputs:velocityCommand"),
            ],
        },
    )
except Exception as e:
    print(e)

try:
    og.Controller.edit(
        {"graph_path": "/Graph/Publish_Joints", "evaluator_name": "execution"},
        {
            og.Controller.Keys.CREATE_NODES: [
                ("OnImpulseEvent", "omni.graph.action.OnImpulseEvent"),
                ("Context", "omni.isaac.ros2_bridge.ROS2Context"),
                ("ReadSimulationTime", "omni.isaac.core_nodes.IsaacReadSimulationTime"),
                ("PublishJointState", "omni.isaac.ros2_bridge.ROS2PublishJointState"),
                ("PublishClock", "omni.isaac.ros2_bridge.ROS2PublishClock")
            ],         
            og.Controller.Keys.SET_VALUES: [
                ("PublishJointState.inputs:nodeNamespace", f"/{NAMESPACE}"), 
                ("PublishJointState.inputs:topicName", "isaac_joint_states"), 
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
        {"graph_path": "/Graph/Publish_Odom", "evaluator_name": "execution"},
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
                ("PublishOdometry.inputs:nodeNamespace", f"/{NAMESPACE}"),
                ("PublishOdometry.inputs:topicName", "odom"),
                ("PublishOdometry.inputs:odomFrameId", f"{NAMESPACE}/odom"),
                ("PublishOdometry.inputs:chassisFrameId", f"{NAMESPACE}/chassis_link"),

                ("PublishOdomTF.inputs:childFrameId", f"{NAMESPACE}/base_link"),
                ("PublishOdomTF.inputs:parentFrameId", f"{NAMESPACE}/odom"),

                #("PublishBaseTF.inputs:childFrameId", "odom"),
                #("PublishBaseTF.inputs:parentFrameId", "world")
            ],
        },
    )
except Exception as e:
    print(e)
    
# Load front camera Realsense D455-------------------------------------------------------------------------------------------
# try:
# 	og.Controller.edit(
#         {   "graph_path": CAMERA_1_GRAPH_PATH, "evaluator_name": "execution" },
#         {
#             og.Controller.Keys.CREATE_NODES: [
#                 ("OnPlaybackTick", "omni.graph.action.OnPlaybackTick"),
#                 ("ROS2Context", "omni.isaac.ros2_bridge.ROS2Context"),
#                 ("RunOneSimulationFrame", "omni.isaac.core_nodes.OgnIsaacRunOneSimulationFrame"),

#                 #Color camera
#                 ("color_camera_render_product",  "omni.isaac.core_nodes.IsaacCreateRenderProduct"),
#                 ("color_camera_publish_rgb", "omni.isaac.ros2_bridge.ROS2CameraHelper"),
#                 ("color_camera_publish_info", "omni.isaac.ros2_bridge.ROS2CameraHelper"),

#                 # Pseudo depth-camera
#                 ("depth_camera_render_product",  "omni.isaac.core_nodes.IsaacCreateRenderProduct"),
#                 ("depth_camera_publish_depth", "omni.isaac.ros2_bridge.ROS2CameraHelper"),
#                 ("depth_camera_publish_info", "omni.isaac.ros2_bridge.ROS2CameraHelper"),

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
#                 ("color_camera_render_product.inputs:cameraPrim", f"{CAMERA_1_STAGE_PATH}/Camera_OmniVision_OV9782_Color"),
                
#                 ("color_camera_publish_rgb.inputs:frameId",  f"{NAMESPACE}/front_camera_link"),
#                 ("color_camera_publish_rgb.inputs:nodeNamespace", f"{NAMESPACE}/front_camera/color"),
#                 ("color_camera_publish_rgb.inputs:topicName", "/image_raw"),
#                 ("color_camera_publish_rgb.inputs:type", "rgb"),

#                 ("color_camera_publish_info.inputs:frameId",  f"{NAMESPACE}/front_camera_link"),
#                 ("color_camera_publish_info.inputs:nodeNamespace", f"{NAMESPACE}/front_camera/color"),
#                 ("color_camera_publish_info.inputs:topicName", "/info"),
#                 ("color_camera_publish_info.inputs:type", "camera_info"),

#                 # Pseudo depth-camera
#                 ("depth_camera_render_product.inputs:cameraPrim",  f"{CAMERA_1_STAGE_PATH}/Camera_Pseudo_Depth"),
                
#                 ("depth_camera_publish_depth.inputs:frameId", f"{NAMESPACE}/front_camera_link"),
#                 ("depth_camera_publish_depth.inputs:nodeNamespace", f"{NAMESPACE}/front_camera/depth"),
#                 ("depth_camera_publish_depth.inputs:topicName", "/image_rect_raw"),
#                 ("depth_camera_publish_depth.inputs:type", "depth"),

#                 ("depth_camera_publish_info.inputs:frameId", f"{NAMESPACE}/front_camera_link"),            
#                 ("depth_camera_publish_info.inputs:nodeNamespace", f"{NAMESPACE}/front_camera/depth"),
#                 ("depth_camera_publish_info.inputs:topicName", "/info"),
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
#                 ("SimulationGate", "omni.isaac.core_nodes.IsaacSimulationGate"),
#                 ("ROS2Context", "omni.isaac.ros2_bridge.ROS2Context"),
#                 ("ReadSimTime", "omni.isaac.core_nodes.IsaacReadSimulationTime"),

#                 ("IsaacReadImu", "omni.isaac.sensor.IsaacReadIMU"),
#                 ("PublishImu", "omni.isaac.ros2_bridge.ROS2PublishImu"),
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
#                 ("IsaacReadImu.inputs:imuPrim",  f"{LIDAR_1_STAGE_PATH}/Imu_Sensor"),
                
#                 ("PublishImu.inputs:frameId", f"{NAMESPACE}/front_camera:imu"),
#                 ("PublishImu.inputs:nodeNamespace", f"{NAMESPACE}/front_camera"),
#                 ("PublishImu.inputs:topicName", "/imu"),

#             ],
#         }
#     )
# except Exception as e:
#     print(e)

# Load front LIDAR-------------------------------------------------------------------------------------------
try:
	og.Controller.edit(
        {   "graph_path": LIDAR_1_GRAPH_PATH, "evaluator_name": "execution" },
        {
            og.Controller.Keys.CREATE_NODES: [
                ("OnPlaybackTick", "omni.graph.action.OnPlaybackTick"),
                ("ROS2Context", "omni.isaac.ros2_bridge.ROS2Context"),
                ("RunOneSimulationFrame", "omni.isaac.core_nodes.OgnIsaacRunOneSimulationFrame"),

                ("lidar_render_product",  "omni.isaac.core_nodes.IsaacCreateRenderProduct"),
                ("lidar_publish_laser_scan", "omni.isaac.ros2_bridge.ROS2RtxLidarHelper"),
                ("lidar_publish_point_cloud", "omni.isaac.ros2_bridge.ROS2RtxLidarHelper"),
            ],
            og.Controller.Keys.CONNECT: [
                ("OnPlaybackTick.outputs:tick", "RunOneSimulationFrame.inputs:execIn"),
                ("RunOneSimulationFrame.outputs:step", "lidar_render_product.inputs:execIn"),

                ("lidar_render_product.outputs:execOut", "lidar_publish_laser_scan.inputs:execIn"),
                ("lidar_render_product.outputs:renderProductPath", "lidar_publish_laser_scan.inputs:renderProductPath"),
                ("ROS2Context.outputs:context", "lidar_publish_laser_scan.inputs:context"),
                
                ("lidar_render_product.outputs:execOut", "lidar_publish_point_cloud.inputs:execIn"),
                ("lidar_render_product.outputs:renderProductPath", "lidar_publish_point_cloud.inputs:renderProductPath"),
                ("ROS2Context.outputs:context", "lidar_publish_point_cloud.inputs:context"),

            ],   
            og.Controller.Keys.SET_VALUES: [
                ("lidar_render_product.inputs:cameraPrim", f"{LIDAR_1_STAGE_PATH}/RPLIDAR_S2E"),
                
                ("lidar_publish_laser_scan.inputs:frameId",  f"{NAMESPACE}/front_laser_laser"),
                ("lidar_publish_laser_scan.inputs:nodeNamespace", f"{NAMESPACE}/front_laser"),
                ("lidar_publish_laser_scan.inputs:topicName", "/laser_scan"),
                ("lidar_publish_laser_scan.inputs:type", "laser_scan"),

                ("lidar_publish_point_cloud.inputs:frameId",  f"{NAMESPACE}/front_laser_laser"),
                ("lidar_publish_point_cloud.inputs:nodeNamespace", f"{NAMESPACE}/front_laser"),
                ("lidar_publish_point_cloud.inputs:topicName", "/point_cloud"),
                ("lidar_publish_point_cloud.inputs:type", "point_cloud"),
                ("lidar_publish_point_cloud.inputs:fullScan", True),
             ],
        },
    )
except Exception as e:
    print(e)

simulation_app.update()
simulation_app.update()

# need to initialize physics getting any articulation..etc
simulation_context.initialize_physics()

simulation_context.play()


while simulation_app.is_running():
    # Run with a fixed step size
    simulation_context.step(render=True)

    # Tick the Publish/Subscribe JointState and Publish Clock nodes each frame
    og.Controller.set(og.Controller.attribute("/Graph/Action_Joints/OnImpulseEvent.state:enableImpulse"), True)
    og.Controller.set(og.Controller.attribute("/Graph/Publish_Joints/OnImpulseEvent.state:enableImpulse"), True)
    og.Controller.set(og.Controller.attribute("/Graph/Publish_Odom/OnImpulseEvent.state:enableImpulse"), True)
    
    
simulation_context.stop()
simulation_app.close()
