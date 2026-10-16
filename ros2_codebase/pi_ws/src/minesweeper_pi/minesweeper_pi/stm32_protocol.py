"""Bounded serial v1 framing shared with firmware/Serial; no ROS dependency."""
import math
import struct
import zlib

SYNC_REQUEST, SYNC_RESPONSE, COMMAND, STATUS, ENCODERS, IMU, METAL = range(1, 8)
MAGIC = b'\xa5\x5a'
MAX_PAYLOAD = 128
HEADER = struct.Struct('<2sBBH')
COMMAND_DATA = struct.Struct('<IIQB5f')
STATUS_DATA = struct.Struct('<IIIIB5fIIII')
ENCODER_DATA = struct.Struct('<QI2q2f')
IMU_DATA = struct.Struct('<QI6f')


def encode(kind, payload=b''):
    if len(payload) > MAX_PAYLOAD:
        raise ValueError('Serial payload too large')
    frame = HEADER.pack(MAGIC, 1, kind, len(payload)) + payload
    return frame + struct.pack('<I', zlib.crc32(frame[2:]))


class Parser:
    def __init__(self):
        self.buffer = bytearray()
        self.errors = 0

    def feed(self, data):
        frames = []
        for value in data:
            self.buffer.append(value)
            while len(self.buffer) >= 2:
                if self.buffer[:2] != MAGIC:
                    del self.buffer[0]
                    continue
                if len(self.buffer) < HEADER.size:
                    break
                _, version, kind, length = HEADER.unpack_from(self.buffer)
                if version != 1 or length > MAX_PAYLOAD:
                    del self.buffer[0]
                    self.errors += 1
                    continue
                total = HEADER.size + length + 4
                if len(self.buffer) < total:
                    break
                crc, = struct.unpack_from('<I', self.buffer, total - 4)
                if crc != zlib.crc32(self.buffer[2:total - 4]):
                    del self.buffer[0]
                    self.errors += 1
                    continue
                frames.append((kind, bytes(self.buffer[HEADER.size:total - 4])))
                del self.buffer[:total]
        return frames


def command_payload(msg, now_ms):
    """Keep original creation time; never make an old ROS command fresh."""
    if (not msg.session or not msg.sequence or not msg.issued_at_ms or
            not -20 <= now_ms - msg.issued_at_ms <= 100 or
            len(msg.target) != 5 or not all(math.isfinite(x) for x in msg.target)):
        raise ValueError('Invalid or expired motor command')
    return COMMAND_DATA.pack(msg.session, msg.sequence, msg.issued_at_ms,
                             bool(msg.arm), *msg.target)
