#!/usr/bin/env bash
set -e

source "/opt/ros/${ROS_DISTRO}/setup.bash"
if [ -f "/ros_ws/install/setup.bash" ]; then
  source "/ros_ws/install/setup.bash"
fi

normalize_ros_name() {
  local value
  value="$(printf '%s' "$1" | tr '[:upper:]' '[:lower:]')"
  value="$(printf '%s' "${value}" | sed -E 's/[^a-z0-9_]+/_/g; s/^_+//; s/_+$//; s/_+/_/g')"
  if [ -z "${value}" ]; then
    value="robot_${ROBOT_ID:-1}"
  fi
  if ! printf '%s' "${value}" | grep -Eq '^[a-zA-Z_]'; then
    value="robot_${value}"
  fi
  printf '%s' "${value}"
}

ROBOT_ID="${ROBOT_ID:-1}"
DEVICE_NAME="${BALENA_DEVICE_NAME_AT_INIT:-${RESIN_DEVICE_NAME_AT_INIT:-${BALENA_DEVICE_NAME:-${DEVICE_NAME:-}}}}"
ROBOT_NAME="${ROBOT_NAME:-${DEVICE_NAME:-robot_${ROBOT_ID}}}"
ROBOT_NAME="$(normalize_ros_name "${ROBOT_NAME}")"
export ROBOT_ID
export ROBOT_NAME
ROS_NAMESPACE="${ROS_NAMESPACE:-${ROBOT_NAME}}"
ROS_NAMESPACE="$(normalize_ros_name "${ROS_NAMESPACE#/}")"
export ROS_NAMESPACE

cat > /tmp/wheelbot_identity.env <<EOF
export ROBOT_ID="${ROBOT_ID}"
export ROBOT_NAME="${ROBOT_NAME}"
export ROS_NAMESPACE="${ROS_NAMESPACE}"
EOF

echo "WheelBot identity: ROBOT_ID=${ROBOT_ID} ROBOT_NAME=${ROBOT_NAME} ROS_NAMESPACE=${ROS_NAMESPACE}"

if [ -n "${ZENOH_ROUTER_ENDPOINT:-}" ] && [ -z "${ZENOH_CONFIG_OVERRIDE:-}" ]; then
  export ZENOH_CONFIG_OVERRIDE="mode=\"client\";connect/endpoints=[\"${ZENOH_ROUTER_ENDPOINT}\"]"
  export ZENOH_ROUTER_CHECK_ATTEMPTS="${ZENOH_ROUTER_CHECK_ATTEMPTS:-0}"
fi

start_front_lidar() {
  local lidar_namespace="/${ROS_NAMESPACE}/front_lidar"
  local serial_port="${RPLIDAR_SERIAL_PORT:-/dev/rplidar}"
  local serial_baudrate="${RPLIDAR_SERIAL_BAUDRATE:-115200}"
  local frame_id="${RPLIDAR_FRAME_ID:-front_lidar_laser}"
  local scan_mode="${RPLIDAR_SCAN_MODE:-Sensitivity}"
  local angle_compensate="${RPLIDAR_ANGLE_COMPENSATE:-true}"

  while true; do
    echo "Starting front RPLIDAR on ${serial_port} in namespace ${lidar_namespace}"
    if ! ros2 run rplidar_ros rplidar_node --ros-args \
      -r "__ns:=${lidar_namespace}" \
      -p "channel_type:=serial" \
      -p "serial_port:=${serial_port}" \
      -p "serial_baudrate:=${serial_baudrate}" \
      -p "frame_id:=${frame_id}" \
      -p "angle_compensate:=${angle_compensate}" \
      -p "scan_mode:=${scan_mode}" \
      -p "topic_name:=scan"; then
      echo "front RPLIDAR process returned an error"
    fi

    echo "front RPLIDAR exited; retrying in ${RPLIDAR_RETRY_SECONDS:-5}s"
    sleep "${RPLIDAR_RETRY_SECONDS:-5}"
  done
}

if [ "${RPLIDAR_ENABLED:-false}" = "true" ]; then
  start_front_lidar &
fi

if [ "$#" -gt 0 ]; then
  exec "$@"
fi

if [ "${NAV2_ENABLED:-false}" != "true" ]; then
  echo "NAV2_ENABLED is not true; keeping nav2 service idle."
  exec sleep infinity
fi

if [ -z "${NAV2_COMMAND:-}" ]; then
  echo "NAV2_ENABLED=true but NAV2_COMMAND is empty; keeping nav2 service idle."
  exec sleep infinity
fi

exec bash -lc "${NAV2_COMMAND}"
