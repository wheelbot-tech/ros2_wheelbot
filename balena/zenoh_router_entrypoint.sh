#!/usr/bin/env bash
set -e

source "/opt/ros/${ROS_DISTRO}/setup.bash"

if [ -n "${ZENOH_REMOTE_ROUTER_ENDPOINT:-}" ] &&
  [ -z "${ZENOH_ROUTER_CONFIG_URI:-}" ] &&
  [ -z "${ZENOH_CONFIG_OVERRIDE:-}" ]
then
  retry_init_ms="${ZENOH_REMOTE_RETRY_INIT_MS:-250}"
  retry_max_ms="${ZENOH_REMOTE_RETRY_MAX_MS:-1000}"
  block_wait_us="${ZENOH_REMOTE_BLOCK_WAIT_US:-1000000}"
  config_template="/etc/wheelbot/zenoh_router_config.json5.in"
  config_file="/tmp/wheelbot_zenoh_router_config.json5"
  sed \
    -e "s|@REMOTE_ENDPOINT@|${ZENOH_REMOTE_ROUTER_ENDPOINT}|g" \
    -e "s|@RETRY_INIT_MS@|${retry_init_ms}|g" \
    -e "s|@RETRY_MAX_MS@|${retry_max_ms}|g" \
    -e "s|@BLOCK_WAIT_US@|${block_wait_us}|g" \
    "${config_template}" > "${config_file}"
  export ZENOH_ROUTER_CONFIG_URI="${config_file}"
fi

exec ros2 run rmw_zenoh_cpp rmw_zenohd
