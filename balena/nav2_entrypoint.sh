#!/usr/bin/env bash
set -e

source "/opt/ros/${ROS_DISTRO}/setup.bash"

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
