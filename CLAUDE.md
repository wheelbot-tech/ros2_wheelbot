# ros2_wheelbot — ROS2 Stack WheelBot

## Despre proiect

Stack ROS2 JKILTED pentru controlul unui AMR omnidirectional WheelBot (vezi wheelbot.tech).
Robotul are **4 virtual joints** (4 wheel + 4 steering) la colturi, controlate prin `topic_based_ros2_control`, si **2 module fizice differential-drive** (fiecare = 1 ESP32 cu 2 motoare ODrive).

## Comenzi esentiale

```bash
cd /home/george/ROS2/ros2_wheelbot
source install/setup.bash

# Build
colcon build --symlink-install

# Build un singur pachet
colcon build --symlink-install --packages-select bringup_mobile

# Launch robot (2 module swerve)
ros2 launch bringup_mobile swerve_2BOT_launch.py

# Teleop
ros2 run teleop_twist_keyboard teleop_twist_keyboard \
  --ros-args --remap cmd_vel:=swerve_controller/cmd_vel -p stamped:=True

# Zenoh router (cu logging)
RUST_LOG=debug ros2 run rmw_zenoh_cpp rmw_zenohd 2>&1 | tee zenoh_$(date +%Y%m%d_%H%M%S).log

# Zenoh bridge DDS
RUST_LOG=debug Z_PAYLOAD_LOG=1 zenoh-bridge-ros2dds

# Bag record
ros2 bag record -a
```

## Lantul de comenzi (arhitectura control)

```
swerve_controller
    → /amr_joint_commands
        → jointstate_to_twist_4
            → /FR_drive_controller/cmd_vel
                → diff_drive_controller
                    → /FR_drive_joint_commands
                        → topic_based_ros2_control
                            → (Zenoh/rmw_zenoh)
                                → FR_picoros_node (ESP32)
```

Acelasi lant exista pentru fiecare modul: FR, FL, RR, RL.

## Topic-uri cheie

| Topic | Tip | Rol |
|-------|-----|-----|
| `/amr_joint_commands` | `JointState` | Comenzi virtuale de la swerve_controller |
| `/amr_joint_states` | `JointState` | Stare agregata toate 4 module (din ESP32) |
| `/FR_drive_joint_commands` | `JointState` | Comenzi fizice modul FR |
| `/FR_drive_joint_states` | `JointState` | Stare encoder modul FR (de la ESP32) |
| `/FR_amr_joint_states` | `JointState` | Stare virtuala modul FR (steering+tractiune) |

Prefix modul: `FR_`, `FL_`, `RR_`, `RL_`

## Structura pachete

| Pachet | Cale | Continut |
|--------|------|----------|
| `bringup_mobile` | `src/mobile_robot/bringup_mobile/` | Launch files, config YAML, noduri C++ |
| `description_mobile` | `src/mobile_robot/description_mobile/` | URDF/Xacro, meshes, rviz |
| `swerve_drive_controller` | `src/wheelbot_ros2_controllers/swerve_drive_controller/` | Controller swerve custom |
| `steering_controllers_library` | `src/wheelbot_ros2_controllers/steering_controllers_library/` | Biblioteca cinematica steering |
| `topic_based_ros2_control` | `src/topic_based_ros2_control/` | Hardware interface prin topic-uri |
| `nav2` | `src/mobile_robot/navigation_ws/nav2/` | Navigatie Nav2 |

## Noduri cheie in bringup_mobile/src/

- `jointstate_aggregator_2.cpp` — centralizeaza starea jointurilor de la 2 module
- `jointstate_aggregator_4.cpp` — varianta pentru 4 module
- `jointstate_to_twist_2.cpp` / `jointstate_to_twist_4.cpp` — transforma comenzi JointState in Twist pentru diff_drive_controller
- `jointstate_aggregator_2_swerve.cpp` — agregator pentru configuratia swerve 2BOT

## Launch files principale

| Launch | Configuratie |
|--------|-------------|
| `swerve_2BOT_launch.py` | Robot real cu 2 module swerve (curent) |
| `swerveBOT_launch.py` | Robot real cu 4 module swerve |
| `control_Layer.launch.py` | Doar control layer |
| `drive_module_launch.py` | Un singur modul de drive |
| `debug_Layer.launch.py` | Debug mode |

## Controllers YAML

Configuratii in `bringup_mobile/config/`:
- `swerve_2BOT_controllers.yaml` — configuratie curenta (2 module)
- `swerveBOT_controllers.yaml` — 4 module

## Problema cunoscuta — swerve steering la zero

`swerve_drive_controller` trimite initial steering joints la zero.

## Simulare Isaac Sim

```bash
ros2 launch isaac_sim_bringup wheelbot_ur_isaac.launch.py
ros2 launch nav2 navigation_wheelbot_sim.launch.py
```

## Environment

- ROS2 Kilted pe Ubuntu 24.04
- Python 3.10+, C++17
- Zenoh ≥ 1.0 / rmw_zenoh
- Workspace: `/home/george/ROS2/ros2_wheelbot`

[jointstate_aggregator_2 arhitectura](project_jointstate_aggregator.md) — `/amr_joint_states` (aggregat de jointstate_aggregator_2) e topic-ul citit de jointstate_to_twist_2, nu cel per-modul.