"""MotorDriver -- the pluggable motor-control block.

Anything that can take two normalized wheel speeds in [-1, 1] and move a
2-wheel differential-drive chassis. Implement this to support a different
H-bridge, an I2C motor HAT (e.g. PCA9685), or anything else -- the rest of
the system (behaviors, safety governor, dashboard) never needs to change.
"""

from abc import ABC, abstractmethod


class MotorDriver(ABC):
    @abstractmethod
    def drive(self, left: float, right: float) -> None:
        """Command wheel speeds, each in [-1.0, 1.0] (negative = reverse)."""

    @abstractmethod
    def stop(self) -> None:
        """Immediately stop both wheels."""

    def close(self) -> None:
        """Release any hardware resources. Default: just stop()."""
        self.stop()
