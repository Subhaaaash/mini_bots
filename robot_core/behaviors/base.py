"""Behavior -- the pluggable vision/decision block.

A Behavior looks at one frame and decides what the robot should do next. It
never talks to motors or GPIO directly -- it returns a BehaviorResult, and
the SafetyGovernor + MotorDriver take it from there. This is the extension
point for line following, obstacle avoidance, ArUco/marker following, color
chasing, leader-follower, etc: implement this interface, register it, done.
See docs/adding_a_behavior.md.
"""

from abc import ABC, abstractmethod
from dataclasses import dataclass, field
from typing import Any, Optional

import numpy as np

from robot_core.drive_command import DriveCommand


@dataclass
class BehaviorResult:
    # None means "no command this frame" (e.g. target lost) -- the
    # SafetyGovernor decides what to do (hold last command briefly, then
    # stop), the Behavior doesn't need its own lost-target policy.
    command: Optional[DriveCommand]
    # Arbitrary JSON-able key/value pairs shown on the dashboard status line.
    telemetry: dict = field(default_factory=dict)
    # Optional annotated frame for the debug MJPEG stream.
    debug_frame: Optional[np.ndarray] = None


class Behavior(ABC):
    """Subclass this to add a new vision/decision algorithm."""

    def reset(self) -> None:
        """Called when the behavior starts/resumes (e.g. after manual
        override ends) so internal state like PID integrators doesn't carry
        stale error across a discontinuity. Default: no-op."""

    @abstractmethod
    def compute(self, frame_bgr: np.ndarray, dt: float) -> BehaviorResult:
        """Process one frame and return the resulting BehaviorResult.

        dt is the wall-clock seconds since the previous call (for PID-style
        behaviors); frame_bgr is an OpenCV BGR image.
        """

    def param_specs(self) -> dict[str, tuple[float, float, float]]:
        """Declare live-tunable params as {name: (min, max, step)} so the
        dashboard can build sliders automatically. Default: none."""
        return {}

    def get_params(self) -> dict[str, Any]:
        return {}

    def set_params(self, params: dict[str, Any]) -> None:
        pass
