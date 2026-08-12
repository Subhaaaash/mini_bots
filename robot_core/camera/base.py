"""CameraSource -- the pluggable frame source.

Anything that can hand back BGR frames (OpenCV's convention) on demand.
Swap in a different camera, or a MockCameraSource fed from a video file/
folder of stills for development and tests without a Pi at all.
"""

from abc import ABC, abstractmethod

import numpy as np


class CameraSource(ABC):
    @abstractmethod
    def start(self) -> None:
        """Open/initialize the source. Safe to call once before the loop."""

    @abstractmethod
    def read(self) -> np.ndarray:
        """Return the next frame as a BGR uint8 array (H, W, 3)."""

    @abstractmethod
    def stop(self) -> None:
        """Release the source."""
