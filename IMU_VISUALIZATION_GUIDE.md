# IMU Motion Visualization Guide

## Overview

Sistemul a fost actualizat pentru a vizualiza mișcările robotului pe baza datelor IMU (accelerometru + giroscop) direct în RViz.

### Ce se vizualizează?

1. **Acceleration Vector (RED)** — Vectorul de accelerație liniară măsurat de senzor
2. **Angular Velocity Vector (GREEN)** — Viteza unghiulară (scaled 10x pentru vizibilitate)
3. **Acceleration Magnitude Sphere (BLUE)** — Sfera dinamică care se redimensionează pe baza magnitudinii accelerației totale
4. **Estimated Orientation Frame (RGB Axes)** — Trei axe colorate (X=Red, Y=Green, Z=Blue) care arată orientarea estimată din filtrul Madgwick

## Fluxul de date

```
ESP32 (wheelbot_espnow_master)
  ↓ serial line
  → WheelbotSerialHardware (parse IMU)
  → /robot_1/imu/data (sensor_msgs/Imu)
  → imu_motion_visualizer (nod nou)
  → /robot_1/imu_markers (visualization_msgs/MarkerArray)
  → RViz (display)
```

## Pornire sistem

### 1. Lansează robotul cu IMU visualization

```bash
cd /home/george/ROS2/ros2_wheelbot
source install/setup.bash

# Lansează robotul cu 2 module swerve (include IMU visualizer automat)
ros2 launch bringup_mobile swerve_2bot_serial.launch.py namespace:=robot_1
```

### 2. Deschide RViz cu configurația actualizată

```bash
ros2 launch description_mobile view_robot_swerve.launch.py \
  namespace:=robot_1 \
  frame_prefix:=robot_1/ \
  rviz_config:=amr_view_robot_1.rviz
```

## Vizualizarea în RViz

Configurația RViz a fost actualizată automat cu:

- ✅ **IMU Display** — Afișează vectorii de accelerație și giroscop 
- ✅ **Marker Array** (`imu_markers`) — Vectorii animați și sfera de magnitudine

### Pasii de configurare manuală (dacă nu apare automat):

1. În RViz → **Add Display** → `rviz_default_plugins/Imu`
   - **Topic:** `/robot_1/imu/data`
   - **Name:** "Imu"

2. În RViz → **Add Display** → `rviz_default_plugins/Marker Array`
   - **Topic:** `/robot_1/imu_markers`
   - **Name:** "IMU Markers"

## Cum să citești vizualizarea

### În timp real pe robot:

- **Red vector (Acceleration)** — Indică direcția accelerației liniare. Lungimea vectorului = magnitudinea accelerației
- **Green vector (Angular Velocity)** — Indică axa de rotație. Lungimea = viteză unghiulară
- **Blue sphere** — Se micșorează/mărește pe baza accelerației totale (util pentru a vedea dacă robotul accelerează)
- **RGB frame** — Arată cum se rotește robotul pe baza estimării din Madgwick filter

### Interpretare misc relativă:

```
Accelerometru (X, Y, Z) = accelerații în frame-ul IMU
  + X = accelerație înainte
  + Y = accelerație laterală (stânga)
  + Z = accelerație verticală (sus)

Giroscop (X, Y, Z) = viteze unghiulare în frame-ul IMU
  + X = roll (rotație pe axa X)
  + Y = pitch (rotație pe axa Y)
  + Z = yaw (rotație pe axa Z — cea mai importantă pentru AMR)
```

## Parametrii filtrului Madgwick (nod imu_motion_visualizer)

Estimarea orientării folosește Madgwick filter cu parametrii:

- **Sample Rate:** 100 Hz (asumă publicare IMU la ~100 Hz)
- **Beta:** 0.05 (control al stabilității vs viteză de convergență)

Dacă vrei să ajustezi acești parametrii, modifică în `src/bringup_mobile/src/imu_motion_visualizer.cpp`:

```cpp
MadgwickFilter filter_(100.0f, 0.05f);  // (sampleRate, beta)
```

Apoi rebuild: `colcon build --symlink-install --packages-select bringup_mobile`

## Troubleshooting

### Nu vejo vectorii în RViz

- Verifică că topicul `/robot_1/imu/data` primește date:
  ```bash
  ros2 topic echo /robot_1/imu/data --once
  ```

- Verifică că display-ul IMU/MarkerArray e **Enabled** în RViz

### Vectorii nu se mișcă

- Asigură-te că robotul ⬅️ **mișcă/accelerează** — vectorii rămân la zero dacă nu e acțiune

### Orientarea estimată nu pare corect

- Filtrul Madgwick are timp de convergență. Mișcă robotul în toate direcțiile (roll, pitch, yaw) pentru ca filtrul să se calibreze

## Noduri relevante

- `imu_motion_visualizer` — Procesează IMU, estima orientare, publică marker-uri
  - Fișier: `src/mobile_robot/bringup_mobile/src/imu_motion_visualizer.cpp`
  - Topic input: `/robot_1/imu/data` (sensor_msgs/Imu)
  - Topic output: `/robot_1/imu_markers` (visualization_msgs/MarkerArray)

- `wheelbot_serial_hardware` — Parsează date serial de ESP32, publică IMU
  - Fișier: `src/mobile_robot/bringup_mobile/src/serial_hardware.cpp`
  - Topic output: `/robot_1/imu/data` (sensor_msgs/Imu)

## Note suplimentare

- Datele IMU au header cu `frame_id = robot_1/imu_link` (configurat din launch file)
- Vectorii sunt re-calculați la fiecare mesaj IMU (latență foarte mică ~10-20ms)
- Marker-urile au lifetime de 0.1s — dacă topicul nu mai primește date, marcajele dispar automat
