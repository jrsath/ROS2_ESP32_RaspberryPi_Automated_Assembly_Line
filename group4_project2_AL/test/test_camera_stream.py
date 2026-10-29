# A stalled stream must not block inspection callbacks or show stale frames.
import threading
import time
import numpy as np
import pytest
from group4_project2_al.camera_stream import CameraStream
from group4_project2_al.camera_window import CameraWindow
from unittest.mock import patch


def test_bad_url_rejected_before_thread_starts():
    with pytest.raises(ValueError, match="stream URL"):
        CameraStream("", "url")


def test_latest_frame_is_copied_and_stale_frames_are_rejected():
    stream = object.__new__(CameraStream)
    stream.lock = threading.Lock()
    stream.frame = np.ones((2, 3, 3), dtype=np.uint8)
    stream.frame_at = time.monotonic()
    stream.stale_seconds = 1.0
    ok, frame = stream.read()
    assert ok
    frame[:] = 0
    assert stream.frame.all()
    stream.frame_at -= 2.0
    assert stream.read() == (False, None)


def test_closed_window_is_not_recreated_by_next_frame():
    window = CameraWindow()
    window.created = True
    with patch("cv2.getWindowProperty", return_value=0), patch("cv2.imshow") as show:
        assert not window.draw(np.zeros((2, 2, 3), dtype=np.uint8))
        show.assert_not_called()


def test_gtk_unsupported_visibility_keeps_live_window_open():
    window = CameraWindow()
    window.created = True
    with patch("cv2.getWindowProperty", side_effect=[-1, 0]):
        assert window.visible()
    with patch("cv2.getWindowProperty", side_effect=[-1, -1]):
        assert not window.visible()


def test_first_frame_opens_and_escape_closes_preview():
    window = CameraWindow()
    with (
        patch("cv2.imshow") as show,
        patch("cv2.waitKey", return_value=27),
        patch("cv2.namedWindow"),
        patch("cv2.resizeWindow"),
    ):
        assert not window.draw(np.zeros((2, 2, 3), dtype=np.uint8))
        show.assert_called_once()


def test_capture_worker_releases_device_on_close():
    class Capture:
        def __init__(self):
            self.released = threading.Event()
            self.reading = threading.Event()

        def isOpened(self):
            return True

        def read(self):
            self.reading.set()
            time.sleep(0.01)
            return True, np.zeros((2, 2, 3), dtype=np.uint8)

        def release(self):
            self.released.set()

    device = Capture()
    with patch("cv2.VideoCapture", return_value=device):
        stream = CameraStream("http://camera/stream", "url")
        assert device.reading.wait(timeout=1)
        stream.release()
        assert not stream.worker.is_alive()
        assert device.released.is_set()
        assert stream.read() == (False, None)
