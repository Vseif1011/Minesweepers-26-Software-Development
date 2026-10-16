"""Native UVC MJPEG capture with v4l2-ctl; no OpenCV decode/encode on the Pi."""
import os
import shutil
import subprocess


class JpegFrames:
    def __init__(self):
        self.buffer = bytearray()

    def feed(self, data):
        self.buffer.extend(data)
        latest = None
        while True:
            start = self.buffer.find(b'\xff\xd8')
            if start < 0:
                self.buffer[:] = self.buffer[-1:]
                break
            if start:
                del self.buffer[:start]
            end = self.buffer.find(b'\xff\xd9', 2)
            if end < 0:
                if len(self.buffer) > 4*1024*1024:
                    self.buffer.clear()
                break
            latest = bytes(self.buffer[:end+2])
            del self.buffer[:end+2]
        return latest


class MjpegCapture:
    def __init__(self, device, width, height, rate):
        executable = shutil.which('v4l2-ctl')
        if executable is None:
            raise RuntimeError('Native MJPEG capture requires v4l-utils (v4l2-ctl)')
        self.frames = JpegFrames()
        self.errors = bytearray()
        self.process = subprocess.Popen([
            executable, '--device', device,
            f'--set-fmt-video=width={width},height={height},pixelformat=MJPG',
            f'--set-parm={rate}', '--stream-mmap=3', '--stream-poll', '--stream-to=-'],
            stdout=subprocess.PIPE, stderr=subprocess.PIPE)
        os.set_blocking(self.process.stdout.fileno(), False)
        os.set_blocking(self.process.stderr.fileno(), False)

    def read_latest(self):
        latest = None
        # A bounded drain drops superseded frames and avoids blocking ROS callbacks.
        for _ in range(32):
            try:
                chunk = os.read(self.process.stdout.fileno(), 65536)
            except BlockingIOError:
                break
            if not chunk:
                break
            image = self.frames.feed(chunk)
            if image is not None:
                latest = image
        try:
            self.errors.extend(os.read(self.process.stderr.fileno(), 8192))
            self.errors[:] = self.errors[-8192:]
        except BlockingIOError:
            pass
        if self.process.poll() is not None:
            raise RuntimeError('Camera capture stopped: '+self.errors.decode(errors='replace'))
        return latest

    def close(self):
        if self.process.poll() is None:
            self.process.terminate()
            try:
                self.process.wait(timeout=2.)
            except subprocess.TimeoutExpired:
                self.process.kill()
                self.process.wait()
        self.process.stdout.close()
        self.process.stderr.close()
