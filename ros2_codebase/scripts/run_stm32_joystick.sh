#!/usr/bin/env bash
set -eo pipefail
root_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
source "/opt/ros/${ROS_DISTRO:-lyrical}/setup.bash"
source "$root_dir/interfaces/install/local_setup.bash"
source "$root_dir/laptop_ws/install/local_setup.bash"
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}"
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export ROS_AUTOMATIC_DISCOVERY_RANGE=SUBNET
unset ROS_LOCALHOST_ONLY
exec ros2 launch minesweeper_laptop joystick_stm32.launch.py "$@"
