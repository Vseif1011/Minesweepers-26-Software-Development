#!/usr/bin/env bash
set -eo pipefail
root_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
ros_distro="${ROS_DISTRO:-lyrical}"
source "/opt/ros/${ros_distro}/setup.bash"
source "$root_dir/interfaces/install/local_setup.bash"
for workspace in pi_ws laptop_ws; do
  (
    cd "$root_dir/$workspace"
    source install/local_setup.bash
    colcon test --python-testing pytest --event-handlers console_direct+ --return-code-on-test-failure
    colcon test-result --verbose
  )
done
source "$root_dir/pi_ws/install/local_setup.bash"
source "$root_dir/laptop_ws/install/local_setup.bash"
python3 "$root_dir/scripts/smoke_test.py"
python3 "$root_dir/scripts/smoke_test_stm32.py"
