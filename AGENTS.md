# AGENTS.md

Instructiuni pentru agentii AI care lucreaza in acest workspace ROS 2.

## Context proiect

`ros2_wheelbot` este workspace-ul ROS 2 Kilted cu middleware rmw_zenoh pentru WheelBot, un AMR omnidirectional modular. Platforma foloseste un strat de control ROS 2 cu `ros2_control`, controllere swerve custom, `topic_based_ros2_control` si comunicatie Zenoh/rmw_zenoh catre module fizice ESP32/ODrive.

Proiectul pentru modulele fizice ESP32/Odrive este : `/home/george/esp/Projects/wheelbot_zenoh-pico` 
Sistemul curent important este varianta swerve cu 2 module fizice differential-drive. Fiecare modul fizic expune stari/comenzi de jointuri, iar nodurile din `bringup_mobile` traduc intre jointurile virtuale swerve si controllerele `diff_drive_controller`.

## Reguli generale pentru agenti

- Lucreaza din radacina workspace-ului: `/home/george/ROS2/ros2_wheelbot`.
- Nu sterge sau rescrie modificari existente fara cerere explicita. Workspace-ul poate avea multe fisiere modificate local.
- Nu modifica directoarele generate `build/`, `install/` sau `log/`.
- Nu comita, nu face push si nu schimba branch-ul decat daca utilizatorul cere explicit.
- Cand modifici launch/config/controller logic, pastreaza compatibilitatea cu ROS 2 Jazzy si C++17.
- Prefer ca modificarile sa fie mici si locale: launch in `launch/`, parametri in `config/`, modele in `urdf/`, cod C++ in `src/` si headere in `include/`.
- Evita refactorizarile largi in pachete vendor/fork daca problema poate fi rezolvata in pachetele WheelBot.

## Structura relevanta

- `src/mobile_robot/bringup_mobile/` - pachetul principal pentru robotul mobil real: launch files, YAML controllers, noduri C++ pentru agregare si conversii `JointState`/`Twist`.
- `src/mobile_robot/description_mobile/` - URDF/Xacro, ros2_control Xacro, meshes si configuratii RViz pentru robotul mobil.
- `src/wheelbot_ros2_controllers/swerve_drive_controller/` - controller swerve custom si cinematica aferenta.
- `src/wheelbot_ros2_controllers/steering_controllers_library/` - biblioteca pentru steering/odometrie folosita de controllere.
- `src/topic_based_ros2_control/` - hardware interface ROS 2 control care comunica prin topicuri.
- `src/isaac_sim_bringup/` - launch, configuratii si asset-uri pentru Isaac Sim.
- `src/mobile_manipulator/` - descriere, bringup si MoveIt pentru varianta mobile manipulator.
- `src/sensors/` si `src/husarion_components_description/` - descrieri si drivere pentru senzori.
- `src/teleop_ws/twist_mux/` - `twist_mux` inclus in workspace.

Exista pachete/fork-uri cu propriul `.git` sub `src/`, de exemplu `rplidar_ros`, `ros_components_description`, `twist_mux`, `topic_based_ros2_control`, `wheelbot_ros2_controllers`, `ur_description` si `husarion_components_description`. Trateaza-le cu grija: nu face operatii git destructive in ele fara acord.

## Build si setup

Comenzi uzuale:

```bash
cd /home/george/ROS2/ros2_wheelbot
source /opt/ros/kilted/setup.bash
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
source install/setup.bash
```

Pentru iteratii mai rapide, construieste pachetul afectat:

```bash
colcon build --symlink-install --packages-select bringup_mobile
colcon build --symlink-install --packages-select description_mobile
colcon build --symlink-install --packages-select swerve_drive_controller
colcon build --symlink-install --packages-select topic_based_ros2_control
```

Daca schimbi un controller sau o biblioteca folosita de alt pachet, construieste si dependentele necesare:

```bash
colcon build --symlink-install --packages-up-to swerve_drive_controller
colcon build --symlink-install --packages-up-to bringup_mobile
```

## Testare

Ruleaza testele doar pentru zona modificata cand este posibil:

```bash
colcon test --packages-select bringup_mobile
colcon test --packages-select description_mobile
colcon test --packages-select swerve_drive_controller
colcon test --packages-select steering_controllers_library
colcon test --packages-select topic_based_ros2_control
colcon test-result --verbose
```

Pachetele de descriere folosesc teste Python/Xacro, iar controllerele folosesc GTest/GMock. Daca nu poti rula testele din cauza dependintelor ROS, noteaza clar comanda incercata si eroarea.

## Launch si operare

Launch principal pentru robotul real cu 2 module swerve:

```bash
ros2 launch bringup_mobile swerve_2BOT_launch.py
```

Alte launch-uri utile:

```bash
ros2 launch bringup_mobile swerveBOT_launch.py
ros2 launch bringup_mobile control_Layer.launch.py
ros2 launch bringup_mobile debug_Layer.launch.py
ros2 launch description_mobile view_robot_swerve.launch.py
ros2 launch isaac_sim_bringup wheelbot_ur_isaac.launch.py
ros2 launch mm_moveit_config mm_sim_moveit.launch.py
```

Teleop uzual:

```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard \
  --ros-args --remap cmd_vel:=swerve_controller/cmd_vel -p stamped:=True
```

Zenoh/rmw_zenoh:

```bash
RUST_LOG=debug ros2 run rmw_zenoh_cpp rmw_zenohd
RUST_LOG=debug Z_PAYLOAD_LOG=1 zenoh-bridge-ros2dds
```

Nu lansa comenzi care pot misca robotul fizic daca utilizatorul nu cere explicit acest lucru si nu confirma contextul hardware.

## Arhitectura control

Lantul principal pentru control swerve este:

```text
swerve_controller
  -> /amr_joint_commands
  -> jointstate_to_twist_2 sau jointstate_to_twist_4
  -> /FR_drive_controller/cmd_vel, /RL_drive_controller/cmd_vel etc.
  -> diff_drive_controller
  -> /FR_drive_joint_commands, /RL_drive_joint_commands etc.
  -> topic_based_ros2_control
  -> Zenoh/rmw_zenoh
  -> noduri ESP32/picoROS
```

Topicuri importante:

- `/amr_joint_commands` - comenzi virtuale de la controllerul swerve.
- `/amr_joint_states` - stare agregata pentru modulele AMR.
- `/FR_drive_joint_commands`, `/RL_drive_joint_commands` etc. - comenzi fizice pe modul.
- `/FR_drive_joint_states`, `/RL_drive_joint_states` etc. - stari encoder/module.
- `/FR_amr_joint_states`, `/RL_amr_joint_states` etc. - stari virtuale per modul.

Prefixele de module folosite in proiect sunt `FR_`, `FL_`, `RR_`, `RL_`.

## Conventii de cod

- C++: foloseste C++17, `rclcpp`, stilul existent din pachet si warning-urile active din CMake.
- Launch Python: pastreaza declaratiile de argumente, remapari si cai `FindPackageShare` clare; evita cai absolute daca poate fi folosit pachetul ROS.
- YAML controllers: modifica nume de controllere, jointuri si topicuri cu atentie; pastreaza sincronizate fisierele launch, URDF/Xacro si config.
- Xacro/URDF: mentine macro-urile parametrizate; verifica numele jointurilor fata de controller YAML si topicurile din bridge.
- RViz/config: evita schimbari masive generate automat daca nu sunt necesare.

## Zone sensibile

- `src/mobile_robot/bringup_mobile/config/*controllers*.yaml`
- `src/mobile_robot/bringup_mobile/launch/*.launch.py`
- `src/mobile_robot/description_mobile/urdf/robots/*.xacro`
- `src/mobile_robot/description_mobile/ros2_control/*.xacro`
- `src/wheelbot_ros2_controllers/swerve_drive_controller/src/*`
- `src/topic_based_ros2_control/src/topic_based_system.cpp`

Cand schimbi aceste zone, verifica macar build-ul pachetului afectat si, daca are sens, `ros2 topic info`/`ros2 topic echo --once` pe topicurile relevante intr-un sistem lansat.

## Debug

Directorul `DEBUG/` contine loguri Zenoh si rosbag-uri MCAP folosite pentru investigatii. Nu le sterge. Pentru analiza MCAP exista scripturi precum:

```bash
python3 DEBUG/analyze_mcap.py
python3 DEBUG/analyze_steering.py
python3 DEBUG/analyze_rl_steering.py
```

Pentru inregistrare:

```bash
ros2 bag record -a
```

## Probleme cunoscute

Exista o problema documentata cu steering-ul swerve la zero: `swerve_drive_controller` trimite initial steering joints la zero, iar unele module pot sa nu ajunga corect la pozitia ceruta. Cand investighezi, coreleaza:

- `/amr_joint_commands`
- `/FR_amr_joint_states` sau modulul afectat
- `/FR_drive_joint_commands` sau modulul afectat
- fisierele MCAP din `DEBUG/rosbag2_2026_02_04-*`

## Cand termini o modificare

- Mentioneaza fisierele schimbate.
- Mentioneaza comenzile de build/test rulate.
- Daca nu ai putut testa, explica scurt de ce.
- Nu include output lung de terminal decat daca este necesar pentru diagnostic.
