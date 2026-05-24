#!/usr/bin/env bash
set -e

source "/opt/ros/${ROS_DISTRO}/setup.bash"

if [ -n "${ZENOH_REMOTE_ROUTER_ENDPOINT:-}" ] && [ -z "${ZENOH_CONFIG_OVERRIDE:-}" ]; then
  export ZENOH_CONFIG_OVERRIDE="connect/endpoints=[\"${ZENOH_REMOTE_ROUTER_ENDPOINT}\"]"
fi

exec ros2 run rmw_zenoh_cpp rmw_zenohd
