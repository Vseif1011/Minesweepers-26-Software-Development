#!/usr/bin/env bash
# Run only the laptop's real YOLO viewer; the Pi camera publisher runs separately.
set -eo pipefail
root_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
vision_env="${MINESWEEPER_VISION_ENV:-$root_dir/artifacts/vision_live/venv}"
source /opt/ros/lyrical/setup.bash
source "$root_dir/interfaces/install/local_setup.bash"
if [[ -f "$root_dir/artifacts/vision_live/ws/install/local_setup.bash" ]]; then
  source "$root_dir/artifacts/vision_live/ws/install/local_setup.bash"
else
  source "$root_dir/laptop_ws/install/local_setup.bash"
fi
if [[ ! -x "$vision_env/bin/python" ]]; then
  echo "Missing vision Python environment: $vision_env" >&2
  exit 1
fi
# ROS entry points built with the system interpreter also need the vision packages.
vision_site="$("$vision_env/bin/python" -c 'import sysconfig; print(sysconfig.get_path("purelib"))')"
export PYTHONPATH="$vision_site${PYTHONPATH:+:$PYTHONPATH}"
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}"
export RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_fastrtps_cpp}"
export ROS_AUTOMATIC_DISCOVERY_RANGE=SUBNET
unset ROS_LOCALHOST_ONLY
export QT_QPA_PLATFORM=xcb
export YOLO_AUTOINSTALL=false
export YOLO_CONFIG_DIR="$root_dir/artifacts/vision_live/ultralytics"
export OMP_NUM_THREADS="${OMP_NUM_THREADS:-4}"
export MKL_NUM_THREADS="${MKL_NUM_THREADS:-4}"
exec ros2 run minesweeper_laptop vision --ros-args \
  -p mock:=false -p display:=true -p device:=cpu -p confidence:=0.25 \
  -p image_size:=416 -p processing_rate:=20.0 \
  -p "model_path:=$root_dir/laptop_ws/src/minesweeper_laptop/models/best.pt" "$@"
