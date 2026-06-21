FROM ros:kilted-ros-base

SHELL ["/bin/bash", "-o", "pipefail", "-c"]
ENV DEBIAN_FRONTEND=noninteractive
ENV ROS_WS=/ros_ws

RUN apt-get update && apt-get install -y --no-install-recommends \
    build-essential \
    cmake \
    python3-colcon-common-extensions \
    ros-${ROS_DISTRO}-navigation2 \
    ros-${ROS_DISTRO}-nav2-bringup \
    ros-${ROS_DISTRO}-rmw-zenoh-cpp \
    ros-${ROS_DISTRO}-sensor-msgs \
    ros-${ROS_DISTRO}-std-srvs \
  && rm -rf /var/lib/apt/lists/*

WORKDIR ${ROS_WS}

COPY src/sensors/rplidar_ros src/sensors/rplidar_ros
COPY src/mobile_robot/navigation_ws/nav2 src/mobile_robot/navigation_ws/nav2
COPY balena/nav2_entrypoint.sh /entrypoint.sh

RUN source /opt/ros/${ROS_DISTRO}/setup.bash \
  && colcon build --merge-install --packages-select rplidar_ros nav2 \
       --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=OFF \
  && chmod +x /entrypoint.sh \
  && rm -rf build log

ENTRYPOINT ["/entrypoint.sh"]
