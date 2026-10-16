# Competition mapping: 20 m × 20 m

The laptop mapper now defaults to **20 columns × 20 rows at 1 m per cell**,
covering all 400 competition squares. This is a semantic mine-location grid.
It is not a SLAM system or a navigation obstacle map. Knowing the field size
fixes the map geometry; it does not measure the robot's position.

## Coordinates and robot geometry

- The map origin is the field's lower-left corner, `(0 m, 0 m)`; +X goes right
  and +Y goes up in the display.
- Cell coordinates are **zero-based**: `column = floor(x / resolution)`,
  `row = floor(y / resolution)`. The first cell is `(0, 0)` and the last is
  `(19, 19)`. `(19.8 m, 19.8 m)` belongs to `(19, 19)`.
- The field includes its lower edges and excludes its upper edges:
  `0 <= x < 20 m`, `0 <= y < 20 m`. Out-of-field detector readings are rejected,
  never clamped into edge cells. Robot positions are not snapped to cell centers.
- Arrays use row-major order: `index = row * width + column`.
- `start_x`, `start_y`, and `start_heading` place the odometry origin in the
  field. They refer to the robot reference point, not the detector. The default
  is `(0 m, 0 m, 0 rad)`, matching the specified bottom-left start facing right. Positive
  heading is counterclockwise from +X. Cell-center corner coordinates are
  `(0.5, 0.5)`, `(19.5, 0.5)`, `(0.5, 19.5)`, `(19.5, 19.5)`.
- `detector_offset` is the detector's distance forward of that robot reference
  point; `detector_lateral_offset` is its distance to the left. Both are metres
  and rotate with the robot. The inherited 0.55 m forward offset is unmeasured.

For odometry pose `(xo, yo, theta)` and configured start `(sx, sy, h)`:

```text
x_robot = sx + cos(h)*xo - sin(h)*yo
y_robot = sy + sin(h)*xo + cos(h)*yo
heading = theta + h
x_detector = x_robot + forward*cos(heading) - left*sin(heading)
y_detector = y_robot + forward*sin(heading) + left*cos(heading)
```

Edit [laptop.yaml](../laptop_ws/src/minesweeper_laptop/config/laptop.yaml)
before launch, then restart the laptop nodes. Both the standalone mapper and
the normal launch use the new dimensions. The static `map → odom` transform
uses the configured start pose.

## Sweep order

The specified route is a serpentine sweep, starting in row 0 at the bottom left:

```text
row 3:  <-------------------
                           ^
row 2:  ------------------->
        ^
row 1:  <-------------------
                           ^  (advance 1 m in map +Y at each row end)
row 0:  start ------------->
```

Even-numbered rows travel right (+X), and odd-numbered rows travel left (-X).
The second row is row 1; repeat through row 19. An advance to the next row means
map +Y regardless of the robot's heading at that end of the row. Row and column
numbers remain fixed field coordinates when travel direction reverses.

The configured `(0 m, 0 m)` start is a metric robot-reference position at the
field origin. Cell `(0, 0)` covers the first square and has its center at
`(0.5 m, 0.5 m)`; these are distinct positions. Adjust `start_x`/`start_y` if the
robot reference point is physically placed at that center instead.

This route is an operator/mapping reference, not an automatic motion controller.
The mapper continues to use measured poses and detector offsets; it does not
force positions onto a row or mark a row searched merely because it was planned.
Physical turning positions must account for the robot and detector geometry
inside the 20 m field.

## Confirmation and missing data

Each arriving positive `/detection` message contributes at most one observation,
using the robot pose available at that arrival. A cell/class needs at least
`confirmation_samples: 3` observations spanning `confirmation_duration: 0.1`
seconds. All must be consecutive observations of the same cell and class.
The reported metric point is their mean detector position. A drawing timer
cannot reuse one reading to create mines along a later robot path.

Negative metal readings, out-of-bounds points, unavailable inputs, a different
candidate cell/class, or a gap of `sensor_timeout: 0.75` seconds break the
candidate sequence. Confirmation reduces isolated spikes and rapid boundary
flicker; it cannot resolve arbitrary localization errors or prolonged false
signals. Short encounters can be missed: tune confirmation time against
measured detector response, robot speed and target size.

Recording requires fresh `/pose_combined`, encoder-backed `/odom`,
`/camera_detection`, and a fresh true `/camera/ready`. The latter is published
by vision only while a successfully processed camera image is fresh.
Missing images therefore do not count as evidence that a mine is buried.
An EKF that continues publishing its held pose cannot hide an odometry outage.
Pose messages must have the configured odometry frame, a valid unit quaternion,
and increasing, recent ROS timestamps. Keep Pi and laptop clocks synchronized.

The existing classification rule is retained: metal plus camera presence means
surface; metal plus a valid camera negative means buried. Surface can replace
buried after confirmation, and is never downgraded. This is one classification
per square, not a count of multiple mines in a square. A mine elsewhere in the
camera image is not geometrically associated with the detector by this rule.

## Display, ROS output, and persistence

The Pygame view fits the 20 × 20 field to the display, labels rows and columns,
and shows the robot, projected detector position, and mine tables. Use the
mouse wheel to scroll the tables and **S** to export JSON plus a screenshot.
Closing the window keeps the headless mapper running.

```bash
ros2 launch minesweeper_laptop laptop.launch.py hardware:=stm32 mock:=false gui:=true
```

That launch's real vision backend requires its optional YOLO dependencies.
If using `scripts/run_live_vision.sh` for the separate model environment, start
the launch with `vision:=false` so only that script publishes vision results.

`/mine_map` is a durable `nav_msgs/OccupancyGrid` with 400 entries:
`0 = unmarked`, `50 = surface`, `100 = buried`. **Unmarked does not mean searched
or clear.** These are semantic labels, not occupancy probabilities; do not
feed this topic directly into a navigation costmap. `/mine_markers` shows the
confirmed metric point inside each cell, and `/mine_pose` emits new/updated
confirmed points.

Each changed cell is saved atomically to `~/.ros/minesweepers/mine_map.json`.
The export retains `width`, `height`, `resolution`, `frame_id`, `cells`,
`surface`, and `buried`, and adds metric field dimensions, coordinate conventions,
start pose, detector offsets, confirmation settings, and metric detections.
Internal/exported cell values remain `0/1/2`; ROS grid values remain `0/50/100`.
Manual export remains available:

```bash
ros2 service call /map/save std_srvs/srv/Trigger '{}'
```

Maps are not automatically resumed; each launch starts a new in-memory run.
An idle shutdown preserves an existing export rather than overwriting it with
an empty map. A run that records new mines replaces that export; archive results
or change `output_path` when retaining multiple runs.

## What still determines field accuracy

There is no demonstrated field-position accuracy yet. The current STM32 encoder
channel assignments and tick scales have not been commissioned, and the MPU-6050
test established working raw sensor data rather than a calibrated global heading.
The bundled YOLO checkpoint's class is named `cube`; its fitness for the actual
competition targets must be verified before trusting classifications.

Commission these against measured field references:

1. Fix the map's physical origin and start pose; measure forward/left detector
   offsets from the same reference used by wheel odometry.
2. Confirm encoder channel assignments/signs and measure left/right ticks per
   metre over known distances, then calibrate effective track width with turns.
3. Secure the IMU, establish the chip-to-robot mounting rotation, and calibrate
   gyro bias. The current EKF consumes chip Z gyro directly, so arbitrary mounting
   is not automatically corrected.
4. Drive measured straight paths, turns and a closed loop while checking estimated
   positions at known grid points. Record maximum position and heading errors,
   not just whether the robot returns near its starting point.
5. Validate detector threshold, confirmation settings, model classes and
   camera/detector alignment with known targets in the first, middle and last
   rows/columns and near cell boundaries.

A persistent 1° heading error gives about **0.35 m lateral error over a straight
20 m run**; a 1% distance-scale error gives **0.20 m** distance error. There is
less tolerance near a cell edge. Wheel/IMU dead reckoning accumulates drift;
reliable global accuracy needs measured corrections from field references,
such as surveyed visual markers or another absolute localization source.
The mapper does not infer those corrections merely from the existence of grid
lines. Boolean detection messages also lack capture timestamps, so arrival-time
association has a remaining motion/latency error. Stamped detections and measured
sensor latency are needed to improve that further.

## Verification

`test_mapping.py` checks all 400 cells, exact boundaries, rotated detector offsets,
configurable resolution, confirmation and interruption behavior, stale/replayed
poses, lost odometry, unavailable vision, JSON exports and idle-run preservation.
The ROS smoke test also checks the live 20 × 20 grid and verifies that moving
with metal detection during a camera outage cannot invent buried cells.
These are software checks using synthetic data, not a physical field-accuracy test.

Validated on 2026-10-17 with `bash scripts/test.sh`: **93 unit tests passed**
(35 Pi, 58 laptop), plus **26 mock ROS integration checks** and **6 synthetic
STM32 telemetry checks**. The rebuilt laptop package passed. A Pygame preview
rendered at 882 × 605 pixels on a simulated 1024 × 768 display; the axes and all
four corner cells were visually checked. Results are in
`artifacts/smoke_report.json`, `artifacts/stm32_integration_report.json`, and
`artifacts/competition_mapping/map_preview.png`.
