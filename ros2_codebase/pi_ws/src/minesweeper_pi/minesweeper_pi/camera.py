"""JPEG-over-DDS replacement for the hardcoded IP/socket camera streamer."""
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import Bool

from .runtime import param, run


class Camera(Node):
    def __init__(self):
        super().__init__('camera')
        self.mock = param(self, 'mock', True)
        self.backend = param(self, 'capture_backend', 'opencv')
        if self.mock or self.backend == 'opencv':
            import cv2
            import numpy as np
            self.cv2, self.np = cv2, np
        self.width = param(self, 'width', 640)
        self.height = param(self, 'height', 480)
        self.quality = param(self, 'jpeg_quality', 40)
        self.frame = param(self, 'frame_id', 'camera_optical_frame')
        self.visible = param(self, 'mock_mine_visible', True)
        rate = param(self, 'rate', 10.0)
        if min(self.width, self.height, rate) <= 0 or not 1 <= self.quality <= 100:
            raise ValueError('invalid camera dimensions, rate, or JPEG quality')
        self.cap = None
        self.mjpeg = None
        if not self.mock and self.backend == 'mjpeg':
            from .mjpeg_capture import MjpegCapture
            device = param(self, 'device_path', '/dev/video0')
            self.mjpeg = MjpegCapture(device, self.width, self.height, rate)
            self.get_logger().info(f'Native MJPEG camera: {device}, {self.width}x{self.height}, {rate} FPS')
        elif not self.mock and self.backend == 'opencv':
            self.cap = cv2.VideoCapture(param(self, 'device', 0))
            if not self.cap.isOpened():
                raise RuntimeError('Unable to open camera')
            self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.width)
            self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.height)
            self.cap.set(cv2.CAP_PROP_FPS, rate)
            self.cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
        elif not self.mock:
            raise ValueError('capture_backend must be opencv or mjpeg')
        self.pub = self.create_publisher(CompressedImage, 'camera/image/compressed', qos_profile_sensor_data)
        if self.mock:
            self.create_subscription(Bool, 'mock/camera_mine_visible', self.set_visible, 10)
        self.create_timer(1.0 / rate, self.update)

    def set_visible(self, msg):
        self.visible = msg.data

    def update(self):
        if self.mjpeg is not None:
            jpeg = self.mjpeg.read_latest()
            if jpeg is not None:
                msg = CompressedImage()
                msg.header.stamp = self.get_clock().now().to_msg()
                msg.header.frame_id = self.frame
                msg.format, msg.data = 'jpeg', jpeg
                self.pub.publish(msg)
            return
        if self.mock:
            frame = self.np.zeros((self.height, self.width, 3), dtype=self.np.uint8)
            if self.visible:
                self.cv2.circle(frame, (self.width//2, self.height//2), min(self.width, self.height)//8,
                                (0, 0, 255), -1)
        else:
            valid, frame = self.cap.read()
            if not valid:
                self.get_logger().warning('Camera frame unavailable')
                return
        valid, jpeg = self.cv2.imencode('.jpg', frame, [self.cv2.IMWRITE_JPEG_QUALITY, self.quality])
        if valid:
            msg = CompressedImage()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = self.frame
            msg.format, msg.data = 'jpeg', jpeg.tobytes()
            self.pub.publish(msg)

    def destroy_node(self):
        if self.cap is not None:
            self.cap.release()
        if self.mjpeg is not None:
            self.mjpeg.close()
        return super().destroy_node()


def main(args=None):
    run(Camera, args)
