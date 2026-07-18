#!/usr/bin/env bash

set -e

_quick_source_is_sourced() {
  [[ "${BASH_SOURCE[0]}" != "${0}" ]]
}

if ! _quick_source_is_sourced; then
  echo "Please run: source quick_source.sh"
  exit 1
fi

_quick_source_script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
_quick_source_ros_setup="/opt/ros/jazzy/setup.bash"
_quick_source_livox_setup="/home/goose/fastlio/livox_ws/install/setup.bash"
_quick_source_local_setup="${_quick_source_script_dir}/install/setup.bash"

if [[ ! -f "${_quick_source_ros_setup}" ]]; then
  echo "Missing ROS setup: ${_quick_source_ros_setup}"
  return 1
fi

if [[ ! -f "${_quick_source_livox_setup}" ]]; then
  echo "Missing Livox workspace setup: ${_quick_source_livox_setup}"
  return 1
fi

if [[ ! -f "${_quick_source_local_setup}" ]]; then
  echo "Missing local workspace setup: ${_quick_source_local_setup}"
  echo "Build first: colcon build --packages-select small_point_lio"
  return 1
fi

source "${_quick_source_ros_setup}"
source "${_quick_source_livox_setup}"
source "${_quick_source_local_setup}"

echo "Sourced:"
echo "  ${_quick_source_ros_setup}"
echo "  ${_quick_source_livox_setup}"
echo "  ${_quick_source_local_setup}"
