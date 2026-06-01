# AGENTS.md

Instructiuni pentru agentii AI care lucreaza in acest workspace ROS 2.

## Context proiect

`ros2_wheelbot` este workspace-ul ROS 2 Kilted cu middleware rmw_zenoh pentru WheelBot, un AMR omnidirectional modular. Platforma foloseste un strat de control ROS 2 cu `ros2_control`, controllere swerve custom si comunicatie seriala catre masterul ESP-NOW care controleaza modulele fizice ESP32/ODrive.

Proiectele firmware ESP32/ODrive active sunt:

- `/home/george/ESP-IDF/PROIECTE/wheelbot_espnow_master` - masterul ESP-NOW montat pe sasiul WheelBot. Face bridge USB serial intre Jetson/laptop si modulele fizice, trimite comenzi `VEL`/`ESTOP` prin ESP-NOW, primeste `JOINT_STATE`, publica pe serial `STATE ...` si publica IMU-ul de sasiu ca `IMU MASTER ...`.
- `/home/george/ESP-IDF/PROIECTE/wheelbot_espnow_m5stack_module` - firmware-ul modulelor M5Stack/ODrive. Fiecare modul primeste `VEL_CMD` prin ESP-NOW, controleaza local ODrive/TWAI si trimite feedback `JOINT_STATE` inapoi la master. Modulele nu mai publica date IMU utile; IMU de pe module este dezactivat/ignorat.

Sistemul curent important este varianta swerve cu 2 module fizice differential-drive (`FR` si `RL`). In `bringup_mobile`, `WheelbotSerialHardware` comunica direct cu masterul ESP-NOW prin `/dev/ttyACM0`, traduce jointurile virtuale swerve in comenzi seriale `VEL FR ...` / `VEL RL ...`, consuma liniile `STATE FR ...` / `STATE RL ...` si publica un singur IMU de sasiu din `IMU MASTER ...`.

## Reguli generale pentru agenti

- Lucreaza din radacina workspace-ului: `/home/george/ROS2/ros2_wheelbot`.
- Pentru inspectie, diagnostic si operare runtime pe sistemul WheelBot curent cu `rmw_zenoh`, foloseste cu prioritate connectorul Zenoh MCP (`mcp__zenoh`). Interogheaza mai intai read-only admin/liveliness/subscribers, de exemplu `zenoh_admin_query`, `zenoh_liveliness_get` si `zenoh_list_subscribers`.
- Porneste local bridge-ul Zenoh MCP inainte de sesiunea Codex/VSCode:
  `zenoh-bridge-mcp --connect tcp/127.0.0.1:7447 --mcp-listen 127.0.0.1:8808`.
  Configuratia Codex trebuie sa pointeze connectorul `zenoh` la `http://127.0.0.1:8808/mcp`.
- Foloseste `ros-mcp-server` doar ca fallback pentru sisteme ROS/rosbridge care sunt vizibile direct prin el. Daca `ros-mcp-server` raporteaza `No nodes found`/`No topics found`, dar Zenoh MCP vede chei `robot_1/...`, considera Zenoh MCP sursa corecta pentru graful activ.
- Apeleaza la comenzi shell (`ros2 node list`, `ros2 topic echo`, `ros2 launch`, `colcon` etc.) doar cand MCP-urile nu acopera operatia, cand ai nevoie de output brut, sau cand utilizatorul cere explicit comanda.
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
- `src/topic_based_ros2_control/` - hardware interface ROS 2 control care comunica prin topicuri; util pentru variantele topic-based/Zenoh mai vechi si sim, nu pentru fluxul serial ESP-NOW curent.
- `balena/` si `docker-compose.yml` - deployment Balena pentru Jetson Nano, cu servicii separate `zenoh_router`, `base_control`, `nav2`, `rmf_agent`.
- `src/isaac_sim_bringup/` - launch, configuratii si asset-uri pentru Isaac Sim.
- `src/mobile_manipulator/` - descriere, bringup si MoveIt pentru varianta mobile manipulator.
- `src/sensors/` si `src/husarion_components_description/` - descrieri si drivere pentru senzori.
- `src/teleop_ws/twist_mux/` - `twist_mux` inclus in workspace.

Exista pachete/fork-uri cu propriul `.git` sub `src/`, de exemplu `rplidar_ros`, `ros_components_description`, `twist_mux`, `topic_based_ros2_control`, `wheelbot_ros2_controllers`, `ur_description` si `husarion_components_description`. Trateaza-le cu grija: nu face operatii git destructive in ele fara acord.

## Build si setup

Pentru informatii despre graful ROS runtime pe robotul activ, prefera Zenoh MCP (`mcp__zenoh`). Pentru informatii de pachete/build local foloseste in continuare `colcon` si toolchain-ul ROS local.

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

Cand un sistem ROS este deja lansat prin `rmw_zenoh`, foloseste mai intai Zenoh MCP pentru inspectie de topicuri/noduri/parametri deduse din admin space/liveliness. Ruleaza `ros-mcp-server` doar ca fallback daca vede acelasi graf, iar comenzi `ros2 ...` manual doar pentru verificari care cer output CLI exact.

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
ros2 launch bringup_mobile swerve_2bot_serial.launch.py
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

Pe Balena, serviciul `base_control` lanseaza implicit `bringup_mobile swerve_2bot_serial.launch.py`. Namespace-ul ROS trebuie sa vina din numele device-ului Balena, de exemplu `robot_1`, iar `frame_prefix` devine `robot_1/`.

Vizualizarea RViz a robotului real namespaced trebuie sa pastreze separate trei lucruri care altfel se amesteca usor:

- Joint names din `/robot_1/joint_states` raman neprefixate (`virtual_front_right_steering_joint`, `virtual_rear_left_wheel_joint` etc.). Nu genera URDF-ul real cu `tf_prefix:=robot_1/`, pentru ca `robot_state_publisher` nu va mai potrivi jointurile reale.
- Pentru TF prefixat foloseste `frame_prefix:=robot_1/` la `robot_state_publisher`; acesta publica frame-uri ca `robot_1/base_footprint`, `robot_1/chassis_link`, `robot_1/FR_steering_link`.
- Pentru RobotModel in RViz, campul `TF Prefix` se scrie fara slash final: `robot_1`, in timp ce `Fixed Frame` ramane cu slash: `robot_1/base_footprint`. Daca `TF Prefix` este `robot_1/`, RViz poate cauta frame-uri de forma `robot_1//...` si raporteaza `No transform`.
- Pentru robotul real, foloseste un `joint_state_publisher` non-GUI ca expander de mimic joints: citeste `/robot_1/joint_states` si publica un topic local expandat, de exemplu `/robot_1/description_joint_states`, pe care il consuma `robot_state_publisher`. Asta adauga mimic joints precum `FR_steering_joint` si `RL_steering_joint`.
- Izoleaza topicurile locale de vizualizare fata de cele publicate de robotul real: foloseste `/robot_1/tf`, `/robot_1/tf_static`, `/robot_1/description_joint_states` si `/robot_1/description_robot_description`, nu topicuri globale sau topicul real `/robot_1/robot_description`.

Comanda verificata pentru vizualizarea robotului real namespaced:

```bash
ros2 launch description_mobile view_robot_swerve.launch.py \
  namespace:=robot_1 \
  frame_prefix:=robot_1/ \
  rviz_config:=amr_view_robot_1.rviz
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

Lantul principal curent pentru control swerve real, cu ESP-NOW master, este:

```text
base_control container / ROS 2
  -> swerve_controller
  -> WheelbotSerialHardware
  -> USB serial /dev/ttyACM0
  -> wheelbot_espnow_master
  -> ESP-NOW VEL_CMD / ESTOP
  -> wheelbot_espnow_m5stack_module FR/RL
  -> ODrive/TWAI local

feedback:
  wheelbot_espnow_m5stack_module FR/RL
  -> ESP-NOW JOINT_STATE
  -> wheelbot_espnow_master
  -> serial STATE FR/RL ...
  -> WheelbotSerialHardware
  -> ros2_control state interfaces

IMU:
  wheelbot_espnow_master fixat pe sasiu
  -> serial IMU MASTER ...
  -> WheelbotSerialHardware
  -> /imu/data sau /<robot_namespace>/imu/data
```

Topicuri importante:

- `/robot_1/swerve_controller/cmd_vel` - comanda de viteza pentru robot cand ruleaza namespaced.
- `/robot_1/joy`, `/robot_1/joy_vel`, `/robot_1/cmd_vel_out` - lantul F710/teleop/twist_mux in Balena.
- `/robot_1/joint_states` - stari ROS 2 control publicate de `joint_state_broadcaster`.
- `/robot_1/imu/data` - IMU unic de sasiu publicat din `IMU MASTER ...`; `frame_id` trebuie sa fie `robot_1/imu_link` cand `frame_prefix=robot_1/`.
- `/amr_joint_commands`, `/amr_joint_states`, `/FR_drive_joint_commands`, `/RL_drive_joint_commands`, `/FR_amr_joint_states`, `/RL_amr_joint_states` pot exista in variantele topic-based/Zenoh mai vechi, dar nu reprezinta fluxul serial ESP-NOW curent.

Prefixele de module folosite in proiect sunt `FR_`, `FL_`, `RR_`, `RL_`.

Protocol serial ESP-NOW master:

- Comenzi trimise de ROS catre master: `VEL <module> <right_rad_s> <left_rad_s>` si `ESTOP <module>`.
- Feedback primit de ROS: `STATE <module> <pos_right> <pos_left> <vel_right> <vel_left> <steering>`.
- IMU primit de ROS: `IMU MASTER <accel_xyz> <gyro_xyz> <vel_right> <vel_left> <steering> <timestamp_ms> <seq>`.
- Nu procesa IMU de module `FR`/`RL` ca sursa de orientare/sasiu; acestea sunt istorice sau dezactivate.

Cand schimbi firmware ESP-IDF:

- In repo-urile ESP, citeste intai `README.md` si `AGENTS.md` local.
- Pentru build/flash/monitor ESP-IDF, foloseste MCP-ul `esp-idf-eim` cand este disponibil.
- Utilizatorul prefera sa faca build manual pentru firmware; nu rula `idf.py build`, `flash` sau `monitor` din chat decat daca cere explicit.
- Mediul ESP-IDF documentat pentru proiectele ESP-NOW este v6.0.1:

```bash
IDF_PYTHON_ENV_PATH=/home/george/.espressif/tools/python/v6.0.1/venv . /home/george/.espressif/v6.0.1/esp-idf/export.sh
```

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

Cand schimbi aceste zone, verifica macar build-ul pachetului afectat si, daca are sens, inspecteaza topicurile/nodurile relevante prin Zenoh MCP (`mcp__zenoh`). Daca Zenoh MCP nu este disponibil sau nu acopera cazul, foloseste `ros-mcp-server` doar daca vede graful, apoi `ros2 topic info`/`ros2 topic echo --once` ca fallback CLI.

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


## Cand termini o modificare

- Mentioneaza fisierele schimbate.
- Mentioneaza comenzile de build/test rulate.
- Daca nu ai putut testa, explica scurt de ce.
- Nu include output lung de terminal decat daca este necesar pentru diagnostic.
