#!/usr/bin/env bash
# Reproduce the Agent source build recorded with the flashed STM32 firmware.
set -eo pipefail
root_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
source /opt/ros/lyrical/setup.bash
agent_workspace="${MINESWEEPER_AGENT_WS:-$root_dir/agent_ws}"
mkdir -p "$agent_workspace/src"
cd "$agent_workspace"
vcs import src < "$root_dir/firmware/stm32f411/gateway/agent.repos"
# System dependencies must be installed first; see docs/STM32_DEPLOYMENT.md.
MAKEFLAGS=-j"${BUILD_JOBS:-1}" colcon build --merge-install --executor sequential \
  --cmake-args -DBUILD_TESTING=OFF
echo "Agent built. Source $agent_workspace/install/local_setup.bash before the real Pi launch."
