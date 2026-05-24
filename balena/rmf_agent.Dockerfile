FROM ros:kilted-ros-base

SHELL ["/bin/bash", "-o", "pipefail", "-c"]
ENV DEBIAN_FRONTEND=noninteractive

RUN apt-get update && apt-get install -y --no-install-recommends \
    python3 \
    ros-${ROS_DISTRO}-rmw-zenoh-cpp \
  && rm -rf /var/lib/apt/lists/*

COPY balena/rmf_agent_entrypoint.sh /entrypoint.sh

RUN chmod +x /entrypoint.sh

ENTRYPOINT ["/entrypoint.sh"]

