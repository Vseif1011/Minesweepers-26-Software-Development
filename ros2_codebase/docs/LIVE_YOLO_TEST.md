# Live Pi camera and laptop YOLO test

The connected camera is a **Logitech Webcam C120**, USB VID/PID `046d:080f`,
serial `D68A4980`. It is connected to the Pi's USB hub path `1-1.3`.
Its image capture device is `/dev/video0`; `/dev/video1` is the companion UVC
node. Use the stable capture path:

```text
/dev/v4l/by-id/usb-046d_080f_D68A4980-video-index0
```

The model is `laptop_ws/src/minesweeper_laptop/models/best.pt`. It was saved by
Ultralytics 8.0.81 and contains **one class: `cube`**. Successful inference does
not establish accuracy against a labeled test set.

```text
Pi Logitech C120 → native MJPEG → ROS 2 /camera/image/compressed
                                      ↓ Wi-Fi
Laptop vision executable → best.pt → boxes / names / confidence → live window
```

The camera’s dynamic-framerate exposure option was disabled for a steadier
requested frame rate. The Pi uses `v4l2-ctl` to capture the camera's native MJPEG, without OpenCV
JPEG decoding/re-encoding. Requested capture is 640×480 at 10 FPS. The camera
node stamps and publishes the compressed frames; inference runs on the laptop.
The laptop publishes `/camera/annotated/compressed`, `/camera_detection` and
`/camera/aligned`. No motor command or navigation nodes are started by this test.

## Start the viewer

From the repository root on the laptop:

```bash
bash scripts/run_live_vision.sh
```

The window is **Minesweepers YOLO — Pi camera**. Point the camera at a cube to
test the trained class. Labels include confidence scores; the top-left overlay
shows processing milliseconds and the inference device. Press **Q**, **Escape**,
or close the window to stop the laptop viewer. The Pi camera publisher remains
running until stopped separately. Start only one viewer instance at a time.

Defaults for this test are CPU inference, image size 416 and confidence 0.25.
The processing timer checks for new frames at 20 Hz; inference runs only when a
new accepted frame is available. The camera remains configured at 10 FPS.
For a different confidence or image size:

```bash
bash scripts/run_live_vision.sh -p confidence:=0.5 -p image_size:=640
```

The isolated Python 3.14 environment is under `artifacts/vision_live/venv` and
uses system OpenCV 4.10 with PyTorch 2.14.1+cpu and Ultralytics 8.4.178. The
isolated ROS overlay under `artifacts/vision_live/ws` points at the repository
package source. The script sources the environments and runs the ROS 2
`minesweeper_laptop vision` executable with `mock:=false display:=true`.

The tested model runs fast enough on CPU for the camera's requested rate. CUDA
was not required for this live test. System OpenCV provides the GUI; the viewer
uses XWayland (`QT_QPA_PLATFORM=xcb`) for a verifiable desktop window. Unused
Ultralytics cloud integrations are not required by this inference path.

## Start the Pi camera after a reboot

The current Pi address is `pi@10.154.60.6`. On the Pi, after sourcing ROS 2 and
`~/minesweeper_serial_ws/install/local_setup.bash`, run:

```bash
export ROS_DOMAIN_ID=0
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export ROS_AUTOMATIC_DISCOVERY_RANGE=SUBNET
ros2 run minesweeper_pi camera --ros-args \
  -p mock:=false -p capture_backend:=mjpeg \
  -p device_path:=/dev/v4l/by-id/usb-046d_080f_D68A4980-video-index0 \
  -p width:=640 -p height:=480 -p rate:=10.0
```

The deployed helper is `~/minesweeper_serial_ws/start_camera_pi.sh`; the running
camera PID and log are under `~/minesweeper_serial_ws/results/camera.pid` and
`camera.log`. The helper executes the installed ROS camera entry point directly
to avoid retaining a ROS CLI wrapper process on the Pi.

Both hosts must be on the same network. Their clocks must be synchronized:
the viewer rejects stale frames and displays a waiting message when the stream
expires. During this test the Pi rebooted with a July clock; it was synchronized
to the laptop over SSH. Ensure NTP works after later Pi reboots.

## Verification

- Camera capture and laptop decoding verified at 640×480.
- Initial stream sample: 161 frames, about 9.83 FPS, median timestamp age 112 ms.
  Timestamp age includes host clock offset and is not a glass-to-glass measurement.
- The real checkpoint loaded and ran on a captured Pi frame. Warm inference was
  approximately 14–17 ms; that sample produced two `cube` predictions at 0.922
  and 0.801 confidence. Live processing also showed around 31 ms on one frame.
- The live viewer window was confirmed `IsViewable` on the laptop desktop.
- An annotated-topic sample received 72 processed frames over approximately
  15 seconds (4.87 FPS), plus 150 detection-state messages, 115 positive.
  The detection-state publication rate is separate from processed frame rate.
- After enabling the 20 Hz processing timer, the viewer itself measured about
  4.7–5.9 processed FPS while receiving about 9.4 frames/second, with no stale-frame
  rejections in that interval. Its latest-frame policy discards superseded frames.
  The GUI remains live; the camera rate and processed frame rate are distinct.
- The laptop/Pi package test suites passed: **62 tests**; the two vision tests
  also passed after adding the processing-rate/FPS diagnostics.

Detailed evidence, launch logs and sample images are in `artifacts/vision_live/`.

The final viewer process exited normally (exit code 0) after the live test.
The Pi camera publisher remained running; use the laptop launch script above
to reopen the viewer.
