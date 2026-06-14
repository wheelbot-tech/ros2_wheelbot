# WheelBot Architecture

## Swerve Controller

```yaml
active_wheels: [front_right, rear_left]  # Only 2 physical modules
```

**Command Interfaces (8 total):**
- 4x wheel velocity: `virtual_*_wheel_joint/velocity`
- 4x steering position: `virtual_*_steering_joint/position`

**State Interfaces (8 total):**
- Same as command interfaces

**Key Features:**
- `cmd_vel_timeout` (5s default) - Stops robot on command loss
- Velocity thresholds per wheel (1.0 m/s default)
- Steering error velocity scaling (cosine-based, 30° threshold)
- Open-loop odometry option
- Chainable controller interface (linear.x, linear.y, angular.z)

## Serial Protocol

**Location:** `bringup_mobile/include/bringup_mobile/serial_protocol.hpp`

### Commands (ROS → ESP-NOW Master)
```
VEL <module> <right_rad_s> <left_rad_s>

ESTOP <module>

```
Modules: `FR`, `FL`, `RR`, `RL` (case-insensitive, trailing `_` stripped)

### Feedback (ESP-NOW Master → ROS)
```
STATE <module> <pos_right> <pos_left> <vel_right> <vel_left> <steering>

IMU MASTER <ax> <ay> <az> <gx> <gy> <gz> <vel_r> <vel_l> <steer> <ts_ms> <seq>

IMUQ MASTER <qx> <qy> <qz> <qw> <gx> <gy> <gz> <ax> <ay> <az> <ts_us> <seq> <status>

JETSON_SHUTDOWN

```

### IMU Status Flags (IMUQ)
| Flag | Value | Meaning |
|------|-------|---------|
| `kImuValid` | 0x01 | IMU data valid |
| `kGyroCalibrated` | 0x02 | Gyro calibrated |
| `kAttitudeValid` | 0x04 | Attitude estimate valid |
| `kAccelCorrectionActive` | 0x08 | Accel correction active |
| `kSampleGap` | 0x10 | Sample gap detected |
| `kImuReadError` | 0x20 | IMU read error |

Orientation published only when `kImuValid | kAttitudeValid` set and no error flags.

## URDF / ros2_control Definition

**Key Files:**
- `swerve_2bot_serial.urdf.xacro` - Main robot entry point
- `swerve_2bot_serial_macro.xacro` - Robot macro with all components
- `wheelbot_serial_swerve_2bot.ros2_control.xacro` - Hardware interface definition

**Joints Defined in ros2_control:**
| Joint | Command | State |
|-------|---------|-------|
| `virtual_front_right_steering_joint` | position | position, velocity |
| `virtual_front_right_wheel_joint` | velocity | position, velocity |
| `virtual_front_left_steering_joint` | position | position, velocity |
| `virtual_front_left_wheel_joint` | velocity | position, velocity |
| `virtual_rear_right_steering_joint` | position | position, velocity |
| `virtual_rear_right_wheel_joint` | velocity | position, velocity |
| `virtual_rear_left_steering_joint` | position | position, velocity |
| `virtual_rear_left_wheel_joint` | velocity | position, velocity |
| `FR_drive_right_joint` | - | position, velocity |
| `FR_drive_left_joint` | - | position, velocity |
| `FR_steering_joint` | - | position, velocity |
| `RL_drive_right_joint` | - | position, velocity |
| `RL_drive_left_joint` | - | position, velocity |
| `RL_steering_joint` | - | position, velocity |

**Virtual vs Module Joints:**
- **Virtual joints** (4 steering + 4 wheel): Swerve kinematics interface, controlled by `swerve_controller`
- **Module joints** (6 per active module): Direct feedback from ESP-NOW modules, read-only

## Namespace & TF Conventions

### Runtime (Balena/Real Robot)
- `ROS_NAMESPACE` = Balena device name (e.g., `robot_1`)
- `frame_prefix` = `robot_1/` (added to TF frames)
- Joint names in `/robot_1/joint_states` **remain unprefixed**
- TF frames: `robot_1/base_footprint`, `robot_1/chassis_link`, `robot_1/FR_steering_link`, etc.
- IMU `frame_id` = `robot_1/imu_link` (when `frame_prefix=robot_1/`)

### RViz Visualization (Namespaced)
```bash
ros2 launch description_mobile view_robot_swerve.launch.py
  namespace:=robot_1
  frame_prefix:=robot_1/
  rviz_config:=amr_view_robot_1.rviz
```

**RViz Settings:**
- `TF Prefix`: `robot_1` (NO trailing slash!)
- `Fixed Frame`: `robot_1/base_footprint`
- Use separate `joint_state_publisher` (non-GUI) to expand mimic joints:
  - Reads `/robot_1/joint_states`
  - Publishes `/robot_1/description_joint_states` (with `FR_steering_joint`, `RL_steering_joint`)
  - `robot_state_publisher` consumes `description_joint_states`

### Topic Isolation
| Real Robot Topic | Visualization Topic |
|------------------|---------------------|
| `/robot_1/joint_states` | `/robot_1/description_joint_states` |
| `/robot_1/robot_description` | `/robot_1/description_robot_description` |
| `/tf` | `/robot_1/tf` |
| `/tf_static` | `/robot_1/tf_static` |

## Teleop & Twist Mux

**Priority Order (highest first):**
1. Joystick (`joy_vel`, priority 100) - F710 gamepad
2. Keyboard (`key_vel`, priority 90) - `teleop_twist_keyboard`
3. Navigation (`cmd_vel_nav`, priority 10) - Nav2

**Locks:**
- `pause_navigation` (priority 100) - Pauses nav, allows joystick
- `stop_closing_loop` (priority 200) - Emergency stop
- `joy_priority` (priority 100) - Joystick priority lock

**Command:**
```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
  --ros-args --remap cmd_vel:=swerve_controller/cmd_vel -p stamped:=True
```

## Odometry Fusion (EKF)

**Configuration:** `ekf_swerve_imu.yaml`

**Inputs:**
- `odom0`: `swerve_controller/odom` (pose x,y,yaw + twist vx,vy,wz)
- `imu0`: `imu/data` (orientation roll,pitch,yaw + angular_vel + linear_accel)

**Fusion:**
- 2D mode enabled
- Publishes `odom` → `base_footprint` TF
- `world_frame` = `odom_frame` = `odom` (no map frame in EKF)

## Build & Development

### Workspace Setup
```bash
cd /home/george/ROS2/ros2_wheelbot
source /opt/ros/kilted/setup.bash
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
source install/setup.bash
```

### Incremental Builds
```bash
colcon build --symlink-install --packages-select bringup_mobile
colcon build --symlink-install --packages-select description_mobile
colcon build --symlink-install --packages-select swerve_drive_controller
colcon build --symlink-install --packages-up-to bringup_mobile
```

### Testing
```bash
colcon test --packages-select bringup_mobile
colcon test --packages-select swerve_drive_controller
colcon test-result --verbose
```

## Runtime Inspection (Zenoh MCP)

**Preferred:** Use `mcp__zenoh` tools for runtime inspection
- `zenoh_admin_query` - Admin space queries
- `zenoh_liveliness_get` - Node liveliness
- `zenoh_list_subscribers` - Topic subscriptions

**Fallback:** `ros2` CLI commands
```bash
ros2 node list
ros2 topic list
ros2 topic echo /robot_1/joint_states --once
ros2 param get /robot_1/swerve_controller wheel_radius
```

## Deployment (Balena)

**Services (docker-compose.yml):**
- `zenoh_router` - Zenoh router
- `base_control` - Runs `bringup_mobile swerve_2bot_serial.launch.py`
- `nav2` - Navigation stack
- `rmf_agent` - RMF fleet agent

**Entrypoint:** `balena/entrypoint.sh` launches `base_control` service

## Firmware Repositories (External)

| Repo | Purpose |
|------|---------|
| `wheelbot_espnow_master` | ESP-NOW master on chassis, USB↔ESP-NOW bridge |
| `wheelbot_espnow_m5stack_module` | Module firmware (ESP32+ODrive), VEL_CMD→JOINT_STATE |

**ESP-IDF Environment:**
```bash
IDF_PYTHON_ENV_PATH=/home/george/.espressif/tools/python/v6.0.1/venv
  . /home/george/.espressif/v6.0.1/esp-idf/export.sh
```

## Debug & Analysis Tools

### Physical Rotation Test

`rotation_test_server.py` is disabled by default on the Jetson. The production
`base_control` service therefore does not expose the remote service and cannot
start an automated rotation test unless it is enabled explicitly.

All commands below can move the physical robot. Clear the area around the
robot, keep an emergency stop available, and use the required `--armed`
argument only after confirming that movement is safe.

#### Remote execution on Jetson, recording on workstation

This variant runs the motion test inside `base_control` on the Jetson and
records the selected ROS topics as MCAP on the workstation.

Temporarily set this Balena device variable and restart `base_control`:

```text
ENABLE_ROTATION_TEST_SERVER=true
```

For a non-Balena launch, the equivalent explicit launch argument is:

```bash
ros2 launch bringup_mobile swerve_2bot_serial.launch.py \
  namespace:=robot_1 \
  frame_prefix:=robot_1/ \
  enable_rotation_test_server:=true
```

Then run on the workstation connected to the same Zenoh graph:

```bash
ros2 run bringup_mobile start_rotation_test.py \
  --namespace robot_1 \
  --armed
```

`start_rotation_test.py` records an MCAP locally, then calls
`/robot_1/start_rotation_test`. The Jetson server starts
`record_rotation_test.py --disable-recording`, so only the workstation creates
the bag. After testing, restore the Balena variable to `false` and restart
`base_control`.

The server can also be started manually in a Jetson shell without restarting
the normal bringup:

```bash
source /opt/ros/kilted/setup.bash
source /ros_ws/install/setup.bash
ros2 run bringup_mobile rotation_test_server.py \
  --ros-args -p namespace:=robot_1
```

Stop that process after the remote test is complete.

#### Direct execution with `record_rotation_test.py`

This variant needs no Jetson test server. Run it on a computer that can see the
namespaced robot topics and can publish to `/robot_1/key_vel`:

```bash
ros2 run bringup_mobile record_rotation_test.py \
  --namespace robot_1 \
  --output-dir DEBUG \
  --armed
```

The process performs the CW/CCW rotation sequence, records MCAP data, and writes
the JSON validation report under `DEBUG/`. Use `--disable-recording` only when
another process already records the required topics.

**MCAP Analysis:**
```bash
python3 DEBUG/analyze_mcap.py
python3 DEBUG/analyze_steering.py
python3 DEBUG/analyze_rl_steering.py
```

**Recording:**
```bash
ros2 bag record -a
```

**Zenoh Debug:**
```bash
RUST_LOG=debug ros2 run rmw_zenoh_cpp rmw_zenohd
RUST_LOG=debug Z_PAYLOAD_LOG=1 zenoh-bridge-ros2dds
```
