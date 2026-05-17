#!/usr/bin/env bash
set -eo pipefail

source /opt/ros/kilted/setup.bash
source /home/george/ROS2/ros2_wheelbot/install/setup.bash

set -u

export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}"
export ROS_LOG_DIR="${ROS_LOG_DIR:-/tmp/ros_logs}"
export RMW_IMPLEMENTATION="${ROS_MCP_RMW_IMPLEMENTATION:-rmw_zenoh_cpp}"
export ROS_AUTOMATIC_DISCOVERY_RANGE="${ROS_AUTOMATIC_DISCOVERY_RANGE:-SUBNET}"

mkdir -p "${ROS_LOG_DIR}"

exec /home/george/.local/bin/uvx ros-mcp --transport=stdio
