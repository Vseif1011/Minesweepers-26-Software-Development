"""Exercise the model inference path without downloading weights or using a camera."""
import sys
from types import SimpleNamespace

import cv2
import numpy as np
import pytest
import rclpy
from sensor_msgs.msg import CompressedImage

from minesweeper_laptop import vision


@pytest.fixture
def node(monkeypatch):
    calls = []

    class Model:
        names = {0: 'mine'}

        def __init__(self, path):
            self.path = path

        def predict(self, frame, **kwargs):
            calls.append(kwargs)

            class Result:
                boxes = SimpleNamespace(xyxy=SimpleNamespace(cpu=lambda:
                                        SimpleNamespace(tolist=lambda: [[10., 20., 80., 100.]])))

                def plot(self, img, **kwargs):
                    calls.append(kwargs)
                    return img

            result = Result()
            result.boxes = BoxSet(result.boxes.xyxy)
            return [result]

    class BoxSet:
        def __init__(self, xyxy):
            self.xyxy = xyxy

        def __len__(self):
            return 1

    monkeypatch.setitem(sys.modules, 'ultralytics', SimpleNamespace(YOLO=Model))
    original = vision.param
    monkeypatch.setattr(vision, 'param', lambda node, name, default:
                        {'mock': False, 'display': False, 'model_path': '/test/model.pt',
                         'device': 'cpu', 'image_size': 320}.get(name, original(node, name, default)))
    rclpy.init()
    instance = vision.Vision()
    yield instance, calls
    instance.destroy_node()
    rclpy.shutdown()


def image(node, age=0):
    ok, jpeg = cv2.imencode('.jpg', np.zeros((120, 160, 3), dtype=np.uint8))
    assert ok
    msg = CompressedImage(format='jpeg', data=jpeg.tobytes())
    timestamp = node.get_clock().now().nanoseconds-int(age*1e9)
    msg.header.stamp.sec, msg.header.stamp.nanosec = divmod(timestamp, 10**9)
    return msg


def test_yolo_predictions_are_annotated_and_published(node):
    instance, calls = node
    detections, annotated = [], []
    instance.pub.publish = detections.append
    instance.annotated_pub.publish = annotated.append
    instance.image(image(instance))
    instance.update()
    assert calls[0]['imgsz'] == 320 and calls[0]['device'] == 'cpu'
    assert calls[1] == {'labels': True, 'conf': True}
    assert detections[-1].data and len(annotated) == 1
    assert cv2.imdecode(np.frombuffer(bytes(annotated[0].data), dtype=np.uint8), cv2.IMREAD_COLOR) is not None


def test_stale_frames_do_not_run_inference(node):
    instance, calls = node
    readiness = []
    instance.ready_pub.publish = readiness.append
    instance.image(image(instance, age=2))
    instance.update()
    assert instance.pending is None and calls == []
    assert not readiness[-1].data


def test_camera_readiness_expires_even_when_negative_flags_keep_publishing(node, monkeypatch):
    instance, _ = node
    readiness, detections = [], []
    instance.ready_pub.publish = readiness.append
    instance.pub.publish = detections.append
    instance.image(image(instance))
    instance.update()
    assert readiness[-1].data and detections[-1].data
    monkeypatch.setattr(instance, 'last_image', float('-inf'))
    instance.update()
    assert not readiness[-1].data and not detections[-1].data
