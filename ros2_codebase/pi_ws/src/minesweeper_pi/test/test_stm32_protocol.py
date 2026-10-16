"""Cross-language framing, corruption recovery and command freshness checks."""
import ctypes
import os
from pathlib import Path
import random
import struct
import subprocess
from types import SimpleNamespace

import pytest
from minesweeper_pi import stm32_protocol as wire


@pytest.fixture(scope='module')
def native(tmp_path_factory):
    default_root = Path(__file__).resolve().parents[4] / 'firmware/stm32f411'
    root = Path(os.environ.get('MINESWEEPER_FIRMWARE_DIR', default_root))
    if not (root/'Serial/Src/serial_protocol.c').exists():
        pytest.skip('Native firmware sources are not part of this standalone Pi workspace')
    output = tmp_path_factory.mktemp('serial') / 'protocol.so'
    subprocess.run(['gcc', '-std=c11', '-Wall', '-Wextra', '-Werror', '-shared', '-fPIC',
                    '-I'+str(root/'Serial/Inc'), str(root/'Serial/Src/serial_protocol.c'),
                    '-o', str(output)], check=True)
    library = ctypes.CDLL(str(output))
    library.serial_encode.argtypes = [ctypes.c_uint8, ctypes.c_void_p, ctypes.c_size_t, ctypes.c_void_p]
    library.serial_encode.restype = ctypes.c_size_t
    return library


class NativeParser(ctypes.Structure):
    _fields_ = [('data', ctypes.c_uint8*138), ('used', ctypes.c_size_t)]


def test_c_python_interoperability_and_fragmented_streams(native):
    rng = random.Random(194)
    callback_type = ctypes.CFUNCTYPE(None, ctypes.c_uint8, ctypes.c_void_p, ctypes.c_size_t, ctypes.c_void_p)
    received = []
    callback = callback_type(lambda kind, data, size, _: received.append((kind, ctypes.string_at(data, size))))
    native.serial_feed.argtypes = [ctypes.POINTER(NativeParser), ctypes.c_void_p, ctypes.c_size_t,
                                   callback_type, ctypes.c_void_p]
    parser, c_parser = wire.Parser(), NativeParser()
    expected = []
    for _ in range(500):
        kind = rng.randrange(1, 8)
        payload = rng.randbytes(rng.randrange(129))
        expected.append((kind, payload))
        frame = wire.encode(kind, payload)
        output = ctypes.create_string_buffer(138)
        length = native.serial_encode(kind, payload, len(payload), output)
        assert output.raw[:length] == frame
        stream = b'noise' + frame
        decoded = []
        while stream:
            size = rng.randrange(1, 24)
            part, stream = stream[:size], stream[size:]
            decoded.extend(parser.feed(part))
            native.serial_feed(ctypes.byref(c_parser), part, len(part), callback, None)
            assert len(parser.buffer) <= 138 and c_parser.used <= 138
        assert decoded == [(kind, payload)]
    assert received == expected


def test_corruption_oversized_headers_and_back_to_back_frames():
    good = wire.encode(wire.STATUS, b'valid')
    bad = bytearray(good)
    bad[6] ^= 1
    oversized = wire.HEADER.pack(wire.MAGIC, 1, wire.STATUS, 65535)
    parser = wire.Parser()
    assert parser.feed(bytes(bad) + oversized + good + good) == [(wire.STATUS, b'valid')]*2
    assert parser.errors >= 2
    with pytest.raises(ValueError):
        wire.encode(wire.STATUS, bytes(129))


@pytest.mark.parametrize('age,valid', [(-21, False), (-20, True), (0, True), (100, True), (101, False)])
def test_command_age_preserves_original_timestamp(age, valid):
    msg = SimpleNamespace(session=7, sequence=9, issued_at_ms=2000-age, arm=False, target=[0.]*5)
    if valid:
        payload = wire.command_payload(msg, 2000)
        assert wire.COMMAND_DATA.unpack(payload)[2] == msg.issued_at_ms
    else:
        with pytest.raises(ValueError):
            wire.command_payload(msg, 2000)


@pytest.mark.parametrize('field,value', [('session', 0), ('sequence', 0), ('issued_at_ms', 0),
                                       ('target', [float('nan')]*5), ('target', [0.]*4)])
def test_invalid_commands(field, value):
    msg = SimpleNamespace(session=7, sequence=9, issued_at_ms=2000, arm=False, target=[0.]*5)
    setattr(msg, field, value)
    with pytest.raises(ValueError):
        wire.command_payload(msg, 2000)
