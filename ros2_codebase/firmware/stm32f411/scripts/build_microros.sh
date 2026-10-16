#!/usr/bin/env bash
set -eo pipefail
root_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
source /opt/ros/lyrical/setup.bash
export TOOLCHAIN_PREFIX="${TOOLCHAIN_PREFIX:-arm-none-eabi-}"
command -v "${TOOLCHAIN_PREFIX}gcc" >/dev/null
python3 "$root_dir/scripts/prepare_microros.py"
# Keep native Python generators available, hide native DDS libraries from CMake.
export PATH="$(python3 -c 'import os;print(":".join(p for p in os.environ["PATH"].split(":") if not p.startswith("/opt/ros/")))')"
export AMENT_PREFIX_PATH="$root_dir/.cache/host_tools"
export CMAKE_PREFIX_PATH="$root_dir/.cache/host_tools"
unset COLCON_PREFIX_PATH
export STATIC_ROSIDL_TYPESUPPORT_C=rosidl_typesupport_microxrcedds_c
export MAKEFLAGS="-j${BUILD_JOBS:-4}"
cd "$root_dir/.cache/client_ws"
colcon build --merge-install --executor sequential \
  --packages-up-to rclc sensor_msgs std_msgs minesweeper_interfaces \
  --packages-ignore-regex '.*_cpp' --metas "$root_dir/micro_ros/colcon.meta" \
  --cmake-clean-cache --cmake-args --no-warn-unused-cli -DCMAKE_POSITION_INDEPENDENT_CODE=OFF \
  -DTHIRDPARTY=ON -DBUILD_SHARED_LIBS=OFF -DBUILD_TESTING=OFF \
  -DCMAKE_BUILD_TYPE=MinSizeRel -DCMAKE_TOOLCHAIN_FILE="$root_dir/micro_ros/toolchain.cmake"
python3 "$root_dir/scripts/prepare_microros.py" --stage
