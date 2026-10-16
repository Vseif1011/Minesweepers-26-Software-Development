#!/usr/bin/env bash
set -eo pipefail
root_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
ros_distro="${ROS_DISTRO:-lyrical}"
source "/opt/ros/${ros_distro}/setup.bash"
# Build shared STM32 message definitions before the application workspaces.
(
  cd "$root_dir/interfaces"
  colcon build --symlink-install --executor sequential "$@"
)
source "$root_dir/interfaces/install/local_setup.bash"
for workspace in pi_ws laptop_ws; do
  (
    cd "$root_dir/$workspace"
    colcon build --symlink-install --executor sequential "$@"
  )
done
