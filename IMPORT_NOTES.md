# Source import

This repository imports the cleaned working tree of
[ejustroboticsclub/Minesweepers_A](https://github.com/ejustroboticsclub/Minesweepers_A),
based on commit `38e055a` on local branch `ros2_branch`, including the subsequent
STM32 direct-serial, Pi camera, YOLO, IMU and 20 m × 20 m mapping work.
It is a fresh source snapshot; the original Git history is not imported.

The current application and build instructions are in
[ros2_codebase/README.md](ros2_codebase/README.md). Historical ROS 1 workspaces
remain in `catkin_ws` and `rpi_ws` for reference.

- Build/install directories, caches, test artifacts, local editor settings and
  Python environments are excluded. Rebuild and install dependencies on each
  machine using the documented scripts. Recorded validation results describe
  checks performed before this source import; local evidence under `artifacts/`
  is not included in Git.
- Firmware vendor libraries are downloaded from the pinned sources in
  `ros2_codebase/firmware/stm32f411/dependencies.json`. Firmware binaries and
  device recovery backups are local artifacts, not part of this source snapshot.
- Both legacy `rosserial` submodules retain commit
  `c169ae2173dcfda7cee567d64beae45198459400`, verified in
  [ros-drivers/rosserial](https://github.com/ros-drivers/rosserial).
  Use `git submodule update --init --recursive` if working on those ROS 1 stacks.
- The original repository also had an empty `wiringPi` checkout with a gitlink
  to `4c5ab96bc251216f509b275cc424e480d57c126e`, without a `.gitmodules` URL.
  That commit could not be resolved in the maintained WiringPi repository, so
  the unusable gitlink is not copied. Legacy code requiring WiringPi needs its
  dependency installed separately; the current STM32 serial path does not use it.

Before import, 93 Python unit tests passed. ROS package discovery, YOLO model
loading and STM32 firmware image checks also passed in the original workspace.
