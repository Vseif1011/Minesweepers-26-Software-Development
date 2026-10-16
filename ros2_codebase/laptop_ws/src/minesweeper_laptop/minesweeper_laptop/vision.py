"""Image subscriber: legacy trained YOLO weights, or synthetic red-mine detector."""
import time
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import Bool

from .runtime import param, run


class Vision(Node):
    def __init__(self):
        super().__init__('vision')
        import cv2
        import numpy as np
        self.cv2, self.np = cv2, np
        self.mock = param(self, 'mock', True)
        self.confidence = param(self, 'confidence', 0.5)
        self.timeout = param(self, 'image_timeout', 0.75)
        self.display = param(self, 'display', False)
        self.flip_vertical = param(self, 'flip_vertical', False)
        self.line_y = param(self, 'line_y', 300)
        self.device = param(self, 'device', 'cpu')
        self.image_size = param(self, 'image_size', 640)
        processing_rate = param(self, 'processing_rate', 10.0)
        if processing_rate <= 0:
            raise ValueError('processing_rate must be positive')
        self.target_classes = param(self, 'target_classes', [-1])
        self.model = None
        if not self.mock:
            from ultralytics import YOLO
            model_path = param(self, 'model_path', str(Path(get_package_share_directory('minesweeper_laptop'))/'models'/'best.pt'))
            self.model = YOLO(model_path)
            self.get_logger().info(f'YOLO model: {model_path}; classes: {self.model.names}; device: {self.device}')
        self.last_image = float('-inf')
        self.detected = False
        self.aligned = False
        self.pending = None
        self.window_name = 'Minesweepers YOLO — Pi camera'
        self.display_frame = None
        self.processed = self.received = self.rejected = 0
        self.rate_started = time.monotonic()
        self.fps = 0.
        if self.display:
            self.cv2.namedWindow(self.window_name, self.cv2.WINDOW_NORMAL)
            self.cv2.resizeWindow(self.window_name, 960, 720)
        self.pub = self.create_publisher(Bool, 'camera_detection', 10)
        self.ready_pub = self.create_publisher(Bool, 'camera/ready', 10)
        self.alignment_pub = self.create_publisher(Bool, 'camera/aligned', 10)
        self.annotated_pub = self.create_publisher(CompressedImage, 'camera/annotated/compressed', qos_profile_sensor_data)
        self.create_subscription(CompressedImage, 'camera/image/compressed', self.image, qos_profile_sensor_data)
        self.create_timer(1.0 / processing_rate, self.update)

    def image(self, msg):
        self.received += 1
        # Drop old frames instead of creating an inference backlog.
        age = (self.get_clock().now().nanoseconds-(msg.header.stamp.sec*10**9+msg.header.stamp.nanosec))/1e9
        if -0.1 <= age <= self.timeout and len(msg.data) <= 4*1024*1024:
            self.pending = msg
        else:
            self.rejected += 1

    def update(self):
        if self.pending is not None:
            msg, self.pending = self.pending, None
            frame = self.cv2.imdecode(self.np.frombuffer(bytes(msg.data), dtype=self.np.uint8), self.cv2.IMREAD_COLOR)
            if frame is not None:
                started = time.monotonic()
                if self.flip_vertical:
                    frame = self.cv2.flip(frame, 0)
                if self.mock:
                    blue, green, red = self.cv2.split(frame)
                    mask = ((red > 150) & (green < 100) & (blue < 100)).astype(self.np.uint8)
                    self.detected = bool(self.np.count_nonzero(mask) > 100)
                    boxes = []
                    if self.detected:
                        x, y, width, height = self.cv2.boundingRect(mask)
                        boxes = [(x, y, x+width, y+height)]
                else:
                    results = self.model.predict(frame, conf=self.confidence, device=self.device, verbose=False,
                                                  imgsz=self.image_size,
                                                  classes=None if -1 in self.target_classes else self.target_classes)
                    self.detected = any(len(result.boxes) > 0 for result in results)
                    boxes = [box for result in results for box in result.boxes.xyxy.cpu().tolist()]
                    frame = results[0].plot(img=frame, labels=True, conf=True)
                line = min(max(self.line_y, 0), frame.shape[0]-1)
                self.aligned = any(box[3] >= line for box in boxes)
                if self.mock:
                    for x1, y1, x2, y2 in boxes:
                        self.cv2.rectangle(frame, (int(x1), int(y1)), (int(x2), int(y2)), (0, 255, 0), 1)
                self.cv2.line(frame, (0, line), (frame.shape[1]-1, line),
                              (0, 255, 0) if self.aligned else (0, 0, 255), 2)
                age = (self.get_clock().now().nanoseconds-(msg.header.stamp.sec*10**9+msg.header.stamp.nanosec))/1e9
                self.last_image = time.monotonic()-max(0., age)
                elapsed = time.monotonic()-started
                self.processed += 1
                rate_elapsed = time.monotonic()-self.rate_started
                if rate_elapsed >= 5.:
                    self.fps = self.processed/rate_elapsed
                    self.get_logger().info(f'Vision: {self.fps:.1f} FPS; received={self.received}, rejected_stale={self.rejected}')
                    self.processed = self.received = self.rejected = 0
                    self.rate_started = time.monotonic()
                self.cv2.putText(frame, f'{"Mock" if self.mock else "YOLO"} | {elapsed*1000:.0f} ms | {self.fps:.1f} FPS | {self.device}',
                                 (10, 24), self.cv2.FONT_HERSHEY_SIMPLEX, .6, (0, 255, 255), 2)
                valid, jpeg = self.cv2.imencode('.jpg', frame)
                if valid:
                    self.annotated_pub.publish(CompressedImage(header=msg.header, format='jpeg', data=jpeg.tobytes()))
                if self.display:
                    self.display_frame = frame
        if self.display:
            frame = self.display_frame
            if time.monotonic()-self.last_image >= self.timeout:
                frame = self.np.zeros((480, 640, 3), dtype=self.np.uint8)
                self.cv2.putText(frame, 'Waiting for live Pi camera stream...', (20, 220),
                                 self.cv2.FONT_HERSHEY_SIMPLEX, .7, (255, 255, 255), 2)
            if frame is not None:
                self.cv2.imshow(self.window_name, frame)
            key = self.cv2.waitKey(1) & 0xff
            if key in (27, ord('q')) or self.cv2.getWindowProperty(self.window_name, self.cv2.WND_PROP_VISIBLE) < 1:
                self.context.try_shutdown()
                return
        ready = time.monotonic()-self.last_image < self.timeout
        self.ready_pub.publish(Bool(data=ready))
        self.pub.publish(Bool(data=self.detected and ready))
        self.alignment_pub.publish(Bool(data=self.aligned and ready))

    def destroy_node(self):
        if self.display:
            self.cv2.destroyAllWindows()
        return super().destroy_node()


def main(args=None):
    run(Vision, args)
