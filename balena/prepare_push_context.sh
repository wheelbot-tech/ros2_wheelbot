#!/usr/bin/env bash
set -euo pipefail

repo_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
target="${1:-/tmp/ros2_wheelbot_balena}"

rm -rf "${target}"
mkdir -p "${target}/src/mobile_robot"
mkdir -p "${target}/src/husarion_components_description"
mkdir -p "${target}/src/sensors"
mkdir -p "${target}/src/wheelbot_ros2_controllers"
mkdir -p "${target}/src/teleop_ws"

cp "${repo_root}/Dockerfile.template" "${target}/"
cp "${repo_root}/docker-compose.yml" "${target}/"
cp "${repo_root}/.dockerignore" "${target}/"
cp -a "${repo_root}/balena" "${target}/balena"
cp -a "${repo_root}/src/mobile_robot/bringup_mobile" "${target}/src/mobile_robot/"
cp -a "${repo_root}/src/mobile_robot/description_mobile" "${target}/src/mobile_robot/"
cp -a "${repo_root}/src/husarion_components_description" "${target}/src/"
cp -a "${repo_root}/src/sensors/rplidar_ros" "${target}/src/sensors/"
cp -a "${repo_root}/src/wheelbot_ros2_controllers/swerve_drive_controller" \
  "${target}/src/wheelbot_ros2_controllers/"
cp -a "${repo_root}/src/teleop_ws/twist_mux" "${target}/src/teleop_ws/"

find "${target}" -type d -name __pycache__ -prune -exec rm -rf {} +
find "${target}" -type f -name '*.pyc' -delete

echo "${target}"
