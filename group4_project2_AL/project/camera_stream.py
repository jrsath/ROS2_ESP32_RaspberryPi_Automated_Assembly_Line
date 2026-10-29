# Read only the newest frame without blocking ROS callbacks or the camera window.
import threading
import time
from pathlib import Path
import cv2


class CameraStream:
    def __init__(self, device, source, stale_seconds=1.0):
        if source not in ("usb", "url"):
            raise ValueError("Camera source must be usb, url or ros")
        if source == "url" and not str(device).startswith(
            ("http://", "https://", "rtsp://")
        ):
            raise ValueError(
                "Set camera.url to the stream URL, for example http://CAMERA_IP:81/stream"
            )
        self.device, self.source = device, source
        self.stale_seconds = float(stale_seconds)
        self.frame = None
        self.frame_at = 0.0
        self.error = "Waiting for camera"
        self.lock = threading.Lock()
        self.stop = threading.Event()
        self.worker = threading.Thread(target=self.run, daemon=True)
        self.worker.start()

    def run(self):
        while not self.stop.is_set():
            capture = None
            try:
                if (
                    self.source == "usb"
                    and isinstance(self.device, int)
                    and not Path(f"/dev/video{self.device}").exists()
                ):
                    self.error = "USB camera not available in Ubuntu"
                else:
                    if self.source == "url":
                        # Bound network waits and keep them off the ROS executor.
                        capture = cv2.VideoCapture(
                            self.device,
                            cv2.CAP_FFMPEG,
                            [
                                cv2.CAP_PROP_OPEN_TIMEOUT_MSEC,
                                2000,
                                cv2.CAP_PROP_READ_TIMEOUT_MSEC,
                                2000,
                            ],
                        )
                    else:
                        capture = cv2.VideoCapture(self.device)
                    if not capture.isOpened():
                        self.error = "Could not open camera stream"
                    else:
                        while not self.stop.is_set():
                            ok, frame = capture.read()
                            if not ok:
                                self.error = "Camera stream disconnected"
                                break
                            with self.lock:
                                self.frame, self.frame_at = frame, time.monotonic()
                            self.error = ""
            except (cv2.error, OSError) as error:
                self.error = str(error)
            finally:
                if capture is not None:
                    capture.release()
                with self.lock:
                    self.frame = None
            self.stop.wait(2.0)

    def read(self):
        with self.lock:
            fresh = (
                self.frame is not None
                and time.monotonic() - self.frame_at < self.stale_seconds
            )
            return (True, self.frame.copy()) if fresh else (False, None)

    def release(self):
        self.stop.set()
        self.worker.join(timeout=2.5)
