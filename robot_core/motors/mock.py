"""A MotorDriver that doesn't touch any hardware -- just records the last
commanded speeds. Used for dev-off-Pi, unit tests, and as the config default
(replaces the old module-level DRY_RUN=True flag: "dry run" is now just
`motor_driver: {type: mock}` in robot.yaml).
"""

from robot_core.motors.base import MotorDriver


class MockDriver(MotorDriver):
    def __init__(self, verbose: bool = True):
        self.verbose = verbose
        self.left = 0.0
        self.right = 0.0
        self.closed = False

    def drive(self, left: float, right: float) -> None:
        self.left = max(-1.0, min(1.0, left))
        self.right = max(-1.0, min(1.0, right))
        if self.verbose:
            print(f"[MockDriver] L={self.left:+.2f} R={self.right:+.2f}")

    def stop(self) -> None:
        self.drive(0.0, 0.0)

    def close(self) -> None:
        self.stop()
        self.closed = True
