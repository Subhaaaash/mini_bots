"""CameraSource backed by Picamera2 (Raspberry Pi Camera Module).

picamera2/libcamera are only importable on a Raspberry Pi with the camera
stack set up, so the import is deferred into __init__ rather than module
top-level, matching the pattern used for the gpiozero motor driver.
"""

import cv2

from robot_core.camera.base import CameraSource


class Picamera2Source(CameraSource):
    def __init__(self, size: tuple[int, int] = (640, 480), hflip: bool = True, vflip: bool = True):
        self.size = size
        self.hflip = hflip
        self.vflip = vflip
        self._picam2 = None

    def start(self) -> None:
        from picamera2 import Picamera2  # deferred: Pi-only dependency
        from libcamera import Transform

        self._picam2 = Picamera2()
        config = self._picam2.create_video_configuration(
            main={"size": self.size, "format": "RGB888"},
            transform=Transform(hflip=int(self.hflip), vflip=int(self.vflip)),
        )
        self._picam2.configure(config)
        self._picam2.start()

    def read(self):
        frame_rgb = self._picam2.capture_array()
        return cv2.cvtColor(frame_rgb, cv2.COLOR_RGB2BGR)

    def stop(self) -> None:
        if self._picam2 is not None:
            self._picam2.stop()
            self._picam2 = None
