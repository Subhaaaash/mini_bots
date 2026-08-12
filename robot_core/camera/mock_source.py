"""A CameraSource with no hardware dependency -- reads frames from a video
file, a folder of image stills (looping), or synthesizes a blank frame if no
path is given. Lets contributors without a Pi run and test Behaviors, and is
what the test suite uses.

A real camera naturally paces the control loop -- read() blocks until the
next frame is ready. This mock reads instantly, so without pacing of its own
the loop would spin as fast as the CPU allows (thousands of iterations/sec),
burning CPU for no benefit and starving other threads (e.g. the dashboard's
Flask server) of GIL time. `fps` reproduces realistic camera timing; pass 0
to disable pacing (e.g. for unit tests that want instant reads).
"""

import time
from pathlib import Path

import cv2
import numpy as np

from robot_core.camera.base import CameraSource


class MockCameraSource(CameraSource):
    def __init__(self, path: str | None = None, size: tuple[int, int] = (640, 480), fps: float = 30.0):
        self.path = Path(path) if path else None
        self.size = size
        self.fps = fps
        self._video = None
        self._frame_paths: list[Path] = []
        self._frame_index = 0
        self._next_frame_time = None

    def start(self) -> None:
        if self.path is None:
            return  # blank-frame mode
        if self.path.is_dir():
            self._frame_paths = sorted(
                p for p in self.path.iterdir() if p.suffix.lower() in (".jpg", ".jpeg", ".png")
            )
            if not self._frame_paths:
                raise FileNotFoundError(f"No image frames found in {self.path}")
        else:
            self._video = cv2.VideoCapture(str(self.path))
            if not self._video.isOpened():
                raise FileNotFoundError(f"Could not open video source {self.path}")

    def _pace(self) -> None:
        if not self.fps:
            return
        now = time.monotonic()
        if self._next_frame_time is None:
            self._next_frame_time = now
        remaining = self._next_frame_time - now
        if remaining > 0:
            time.sleep(remaining)
        self._next_frame_time = max(now, self._next_frame_time) + 1.0 / self.fps

    def read(self) -> np.ndarray:
        self._pace()

        if self.path is None:
            return np.full((self.size[1], self.size[0], 3), 255, dtype=np.uint8)

        if self._video is not None:
            ok, frame = self._video.read()
            if not ok:  # loop back to the start
                self._video.set(cv2.CAP_PROP_POS_FRAMES, 0)
                ok, frame = self._video.read()
            if not ok:
                raise RuntimeError(f"Could not read any frames from {self.path}")
            return frame

        frame_path = self._frame_paths[self._frame_index % len(self._frame_paths)]
        self._frame_index += 1
        frame = cv2.imread(str(frame_path))
        if frame is None:
            raise RuntimeError(f"Could not read frame {frame_path}")
        return frame

    def stop(self) -> None:
        if self._video is not None:
            self._video.release()
            self._video = None
