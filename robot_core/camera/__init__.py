"""Registry of available CameraSource implementations, selected by name from
robot.yaml (`camera: {type: ...}`).
"""

from robot_core.camera.base import CameraSource
from robot_core.camera.mock_source import MockCameraSource
from robot_core.camera.picamera2_source import Picamera2Source

CAMERA_REGISTRY = {
    "mock": MockCameraSource,
    "picamera2": Picamera2Source,
}

__all__ = ["CameraSource", "CAMERA_REGISTRY"]
