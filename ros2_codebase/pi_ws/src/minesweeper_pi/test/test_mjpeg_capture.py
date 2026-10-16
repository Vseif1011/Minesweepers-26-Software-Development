from minesweeper_pi.mjpeg_capture import JpegFrames


def test_fragmented_frames_and_latest_frame_selection():
    frames = JpegFrames()
    assert frames.feed(b'camera status\xff') is None
    assert frames.feed(b'\xd8first\xff') is None
    assert frames.feed(b'\xd9\xff\xd8second\xff\xd9') == b'\xff\xd8second\xff\xd9'
    assert frames.buffer == b''


def test_incomplete_or_noisy_stream_is_bounded():
    frames = JpegFrames()
    assert frames.feed(b'x'*100000) is None
    assert len(frames.buffer) <= 1
    assert frames.feed(b'\xff\xd8'+b'x'*(4*1024*1024+1)) is None
    assert frames.buffer == b''
