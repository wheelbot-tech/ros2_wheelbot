# 🦾 ROS 2 WheelBot

**ROS 2 WheelBot** is an software stack for controlling modular omnidirectional autonomous mobile robots (AMRs).  
It provides a unified control layer for both R&D and industrial variants of the WheelBot platform — from low-level motor control to navigation, perception, and safety integration.

---

## 🚀 Overview

WheelBot is a modular drive system designed to scale from lightweight research robots to heavy-duty industrial AMRs.  
This repository contains the **ROS 2 packages, launch files, and configuration layers** that form the software core of the platform.

### Supported Variants

| Variant | Description | Key Hardware |
|----------|--------------|---------------|
| **WheelBot R&D** | Lightweight omnidirectional robot for research and prototyping | NVIDIA Jetson Orin Nano, Intel RealSense D455, RPLIDAR S2 |
| **WheelBot Industrial** | Heavy-duty AMR with EtherCAT and functional safety | NVIDIA Jetson AGX Thor, Intel RealSense D457, SICK Nanoscan3, EtherCAT FSoE Safety, motor controllers with STO |

---

## 🧩 Architecture

WheelBot uses a modular ROS 2 architecture:
ros2_wheelbot

├── wheelbot_bringup/ → launch and configuration files

├── wheelbot_description/ → URDF/Xacro robot model

├── wheelbot_control/ → ros2_control interfaces and controllers

├── wheelbot_navigation/ → Nav2 and behavior trees

├── wheelbot_perception/ → camera and lidar integration

└── wheelbot_utils/ → helper nodes and diagnostic tools

Each sub-package can be developed, launched, and tested independently.

---

## ⚙️ Features

- **ROS 2** Kilted   
- **ros2_control** actuator interfaces for direct and EtherCAT drives  
- **topic_based_ros2_control** integration for distributed control  
- **Zenoh / rmw_zenoh** and **Agnocast** for zero-copy data sharing  
- **Nav2** navigation with omnidirectional motion model  
- **Realsense + LIDAR** fusion for perception  
- **Safety integration** via EtherCAT FSoE and SICK Nanoscan3  
- **Simulation support** (Isaac Sim)

---

## 🛠️ Installation

```bash
# 1. Clone the repository
git clone https://github.com/wheelbot-tech/ros2_wheelbot.git
cd ros2_wheelbot

# 2. Install dependencies
rosdep install --from-paths src --ignore-src -r -y

# 3. Build the workspace
colcon build --symlink-install

# 4. Source the workspace
source install/setup.bash

---
**Development Notes**
Tested on Ubuntu 24.04 with ROS 2 Kilted
Compatible with Zenoh ≥ 1.0, rmw_zenoh
Recommended Python 3.10+ and C++17
Configurations and parameters are stored in config/ and launch/ folders of each sub-package.
---
