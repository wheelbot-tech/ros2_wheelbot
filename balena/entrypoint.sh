#!/usr/bin/env bash
set -e

source "/opt/ros/${ROS_DISTRO}/setup.bash"
source "${ROS_WS}/install/setup.bash"

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

if [ -n "${ZENOH_ROUTER_ENDPOINT:-}" ] && [ -z "${ZENOH_CONFIG_OVERRIDE:-}" ]; then
  export ZENOH_CONFIG_OVERRIDE="mode=\"client\";connect/endpoints=[\"${ZENOH_ROUTER_ENDPOINT}\"]"
  export ZENOH_ROUTER_CHECK_ATTEMPTS="${ZENOH_ROUTER_CHECK_ATTEMPTS:-0}"
fi

if [ "$#" -gt 0 ]; then
  exec "$@"
fi

LAUNCH_FILE="${LAUNCH_FILE:-swerve_2bot_serial.launch.py}"
SERIAL_PORT="${SERIAL_PORT:-/dev/ttyACM0}"
BAUDRATE="${BAUDRATE:-115200}"
COMMAND_TIMEOUT_MS="${COMMAND_TIMEOUT_MS:-500}"
ACTIVE_MODULES="${ACTIVE_MODULES:-FR,RL}"
JOY_DEV="${JOY_DEV:-/dev/input/js0}"
JOY_CONFIG="${JOY_CONFIG:-F710_sim.yaml}"
ENABLE_ODOM_FUSION="${ENABLE_ODOM_FUSION:-true}"
TF_FRAME_PREFIX="${TF_FRAME_PREFIX:-}"
if [ -z "${TF_FRAME_PREFIX}" ] && [ -n "${ROS_NAMESPACE}" ]; then
  TF_FRAME_PREFIX="${ROS_NAMESPACE#/}/"
fi
export TF_FRAME_PREFIX
export ENABLE_ODOM_FUSION
CONTROLLERS_FILE="${CONTROLLERS_FILE:-/tmp/${ROS_NAMESPACE}_swerve_2bot_serial_controllers.yaml}"
EKF_CONFIG_FILE="${EKF_CONFIG_FILE:-/tmp/${ROS_NAMESPACE}_ekf_swerve_imu.yaml}"
export CONTROLLERS_FILE
export EKF_CONFIG_FILE

cat > /tmp/wheelbot_identity.env <<EOF
export ROBOT_ID="${ROBOT_ID}"
export ROBOT_NAME="${ROBOT_NAME}"
export ROS_NAMESPACE="${ROS_NAMESPACE}"
export TF_FRAME_PREFIX="${TF_FRAME_PREFIX}"
export ENABLE_ODOM_FUSION="${ENABLE_ODOM_FUSION}"
export CONTROLLERS_FILE="${CONTROLLERS_FILE}"
export EKF_CONFIG_FILE="${EKF_CONFIG_FILE}"
EOF

echo "WheelBot identity: ROBOT_ID=${ROBOT_ID} ROBOT_NAME=${ROBOT_NAME} ROS_NAMESPACE=${ROS_NAMESPACE} TF_FRAME_PREFIX=${TF_FRAME_PREFIX}"
echo "WheelBot controllers file: ${CONTROLLERS_FILE}"
echo "WheelBot EKF config file: ${EKF_CONFIG_FILE}"
echo "WheelBot odom fusion: ENABLE_ODOM_FUSION=${ENABLE_ODOM_FUSION}"

cat > "${CONTROLLERS_FILE}" <<EOF
/${ROS_NAMESPACE}/controller_manager:
  ros__parameters:
    update_rate: 30

    joint_state_broadcaster:
      type: joint_state_broadcaster/JointStateBroadcaster

    swerve_controller:
      type: swerve_drive_controller/SwerveController

/${ROS_NAMESPACE}/swerve_controller:
  ros__parameters:
    wheelbase: 0.43
    trackwidth: 0.43
    offset: [0.0, 0.0]
    center_of_rotation: [0.0, 0.0]
    wheel_radius: 0.0825

    cmd_vel_timeout_seconds: 5.0
    use_stamped_vel: true

    front_left_wheel_joint: virtual_front_left_wheel_joint
    front_right_wheel_joint: virtual_front_right_wheel_joint
    rear_left_wheel_joint: virtual_rear_left_wheel_joint
    rear_right_wheel_joint: virtual_rear_right_wheel_joint

    front_left_axle_joint: virtual_front_left_steering_joint
    front_right_axle_joint: virtual_front_right_steering_joint
    rear_left_axle_joint: virtual_rear_left_steering_joint
    rear_right_axle_joint: virtual_rear_right_steering_joint

    front_left_velocity_threshold: 2.0
    front_right_velocity_threshold: 2.0
    rear_left_velocity_threshold: 2.0
    rear_right_velocity_threshold: 2.0

    pose_covariance_diagonal: [0.001, 0.001, 0.001, 0.001, 0.001, 0.01]
    twist_covariance_diagonal: [0.001, 0.001, 0.001, 0.001, 0.001, 0.01]

    odom: odom
    base_footprint: base_footprint
    enable_odom_tf: false
    open_loop: true
EOF

cat > "${EKF_CONFIG_FILE}" <<EOF
/${ROS_NAMESPACE}/ekf_filter_node:
  ros__parameters:
    frequency: 30.0
    sensor_timeout: 0.2
    two_d_mode: true

    publish_tf: true
    publish_acceleration: false
    print_diagnostics: false

    map_frame: map
    odom_frame: ${TF_FRAME_PREFIX}odom
    base_link_frame: ${TF_FRAME_PREFIX}base_footprint
    world_frame: ${TF_FRAME_PREFIX}odom

    odom0: swerve_controller/odom
    odom0_config: [
      true,  true,  false,
      false, false, true,
      true,  true,  false,
      false, false, true,
      false, false, false
    ]
    odom0_queue_size: 10
    odom0_nodelay: true
    odom0_differential: false
    odom0_relative: false

    imu0: imu/data
    imu0_config: [
      false, false, false,
      false, false, false,
      false, false, false,
      false, false, true,
      false, false, false
    ]
    imu0_queue_size: 50
    imu0_nodelay: true
    imu0_differential: false
    imu0_relative: false
    imu0_remove_gravitational_acceleration: false
EOF

exec ros2 launch bringup_mobile "${LAUNCH_FILE}" \
  namespace:="${ROS_NAMESPACE}" \
  frame_prefix:="${TF_FRAME_PREFIX}" \
  controllers_file:="${CONTROLLERS_FILE}" \
  ekf_config_file:="${EKF_CONFIG_FILE}" \
  serial_port:="${SERIAL_PORT}" \
  baudrate:="${BAUDRATE}" \
  command_timeout_ms:="${COMMAND_TIMEOUT_MS}" \
  active_modules:="${ACTIVE_MODULES}" \
  enable_odom_fusion:="${ENABLE_ODOM_FUSION}" \
  joy_dev:="${JOY_DEV}" \
  joy_config:="${JOY_CONFIG}"
