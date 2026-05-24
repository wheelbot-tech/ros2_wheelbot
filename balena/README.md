# Balena Jetson Nano Robot Stack

This profile builds a ROS 2 Kilted multi-service Balena release for the
WheelBot Jetson Nano:

- `base_control`: real robot serial bringup with `ros2_control` and swerve
  controller.
- `zenoh_router`: local Zenoh router used by every robot-side ROS service.
- `nav2`: Nav2 runtime image, idle by default until a launch command is set.
- `rmf_agent`: Open-RMF robot agent/fleet bridge image, idle by default until a
  command is set.

`base_control` compiles only the local packages needed by the serial 2-module
bringup:

- `bringup_mobile`
- `description_mobile`
- `swerve_drive_controller`
- `twist_mux`

The default `base_control` launch starts the serial 2-module bringup:

```bash
ros2 launch bringup_mobile swerve_2bot_serial.launch.py
```

## Deploy

From the workspace root:

```bash
balena login
balena push <your-balena-app-name>
```

Balena Cloud Builder builds the image for the Jetson Nano fleet architecture.

For a ROS workspace that has been built locally, prefer pushing from a clean
Balena context so `build/`, `install/`, `log/`, symlink-install artifacts, and
Python `__pycache__` files are not packaged:

```bash
bash balena/prepare_push_context.sh
balena push <your-balena-app-name> --source /tmp/ros2_wheelbot_balena
```

When adding new Balena services, Dockerfiles, entrypoints, configs, launch
files, or local ROS packages, update `balena/prepare_push_context.sh` in the
same change. The clean context contains only what that script copies, so missing
entries there will build locally but disappear during `balena push --source
/tmp/ros2_wheelbot_balena`.

Direct push from the workspace root is fine after cleaning generated folders:

```bash
rm -rf build install log
balena push <your-balena-app-name>
```

## Shared Runtime Configuration

Set these environment variables in BalenaCloud for all services if needed:

```text
ROBOT_ID=1
ROS_DOMAIN_ID=0
RMW_IMPLEMENTATION=rmw_zenoh_cpp
ZENOH_ROUTER_ENDPOINT=tcp/127.0.0.1:7447
```

`base_control`, `nav2`, and `rmf_agent` derive `ROBOT_NAME` and
`ROS_NAMESPACE` from the Balena device name when `ROBOT_NAME` is not set. The
name is normalized to a ROS-safe value by lowercasing it and replacing invalid
characters with `_`. For example:

```text
Balena device name: robot_1
ROBOT_NAME=robot_1
ROS_NAMESPACE=robot_1
TF_FRAME_PREFIX=robot_1/
```

If Balena does not provide the device name in the container, set `ROBOT_NAME`
explicitly as a device variable.

Balena terminal sessions do not automatically inherit variables exported by the
service entrypoint. To inspect the identity selected by the running service:

```bash
cat /tmp/wheelbot_identity.env
source /tmp/wheelbot_identity.env
echo "$ROBOT_NAME"
echo "$ROS_NAMESPACE"
```

When `ZENOH_ROUTER_ENDPOINT` is set, the entrypoint exports:

```text
ZENOH_CONFIG_OVERRIDE=mode="client";connect/endpoints=["<endpoint>"]
ZENOH_ROUTER_CHECK_ATTEMPTS=0
```

This makes each robot-side ROS service connect to the local Jetson Zenoh router.
Do not point `base_control`, `nav2`, or `rmf_agent` directly at the Open-RMF
computer unless you intentionally want to bypass the local router.

## zenoh_router

The robot runs a local Zenoh router by default. Robot-side services connect to:

```text
ZENOH_ROUTER_ENDPOINT=tcp/127.0.0.1:7447
```

To connect the Jetson router to the Open-RMF computer router, set this only on
the `zenoh_router` service:

```text
ZENOH_REMOTE_ROUTER_ENDPOINT=tcp/<rmf-computer-ip>:7447
```

The Open-RMF computer should also run:

```bash
RMW_IMPLEMENTATION=rmw_zenoh_cpp ros2 run rmw_zenoh_cpp rmw_zenohd
```

## base_control

Service-specific variables:

```text
LAUNCH_FILE=swerve_2bot_serial.launch.py
TF_FRAME_PREFIX=robot_1/
SERIAL_PORT=/dev/ttyACM0
BAUDRATE=115200
COMMAND_TIMEOUT_MS=500
ACTIVE_MODULES=FR,RL
JOY_DEV=/dev/input/js0
JOY_CONFIG=F710_sim.yaml
```

At startup, `base_control` generates a namespace-specific controller YAML in
`/tmp`, for example:

```text
/tmp/robot_1_swerve_2bot_serial_controllers.yaml
```

The generated file uses exact namespaced keys such as
`/robot_1/controller_manager` and `/robot_1/swerve_controller`, avoiding fragile
wildcard parameter matching.

To launch only the headless serial control layer, set:

```text
LAUNCH_FILE=control_Layer_serial_launch.py
```

The service runs privileged so USB serial devices are visible inside the
container. If the ESP master appears as another device, change `SERIAL_PORT`
instead of editing `docker-compose.yml`.

## nav2

`nav2` is installed in its own service, but the service is idle by default:

```text
NAV2_ENABLED=false
NAV2_COMMAND=
```

When the Nav2 config is ready, enable it with a full command, for example:

```text
NAV2_ENABLED=true
NAV2_COMMAND=ros2 launch nav2_bringup bringup_launch.py namespace:=robot_1 use_namespace:=True params_file:=/path/to/nav2_params.yaml
```

Keep `robot_1/odom` and `robot_1/base_link` aligned with the `base_control`
TF/frame setup.

## rmf_agent

`rmf_agent` is also idle by default:

```text
RMF_AGENT_ENABLED=false
RMF_AGENT_COMMAND=
```

When the Open-RMF adapter/robot agent exists, enable it with the command that
starts the WheelBot robot handler:

```text
RMF_AGENT_ENABLED=true
RMF_AGENT_COMMAND=<command that starts the robot_1 RMF agent>
```

Use `ROBOT_NAME=robot_1` as the RMF robot name and keep the fleet name in the
RMF adapter configuration.
