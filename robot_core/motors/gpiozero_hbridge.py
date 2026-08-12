"""MotorDriver backed by gpiozero's Motor class, driving a simple 2-pin-per-
side H-bridge (e.g. an L298N/TB6612-style driver wired to two N20 gearmotors).

gpiozero (and its RPi.GPIO/lgpio backend) is only importable on a Raspberry
Pi, so the import is deferred into __init__ rather than module top-level --
this lets the rest of robot_core import cleanly on any dev machine.
"""

from robot_core.motors.base import MotorDriver


class GpiozeroHBridgeDriver(MotorDriver):
    def __init__(self, left_pins: tuple[int, int], right_pins: tuple[int, int]):
        from gpiozero import Motor  # deferred: Pi-only dependency

        self.left = Motor(forward=left_pins[0], backward=left_pins[1])
        self.right = Motor(forward=right_pins[0], backward=right_pins[1])

    def drive(self, left: float, right: float) -> None:
        left = max(-1.0, min(1.0, left))
        right = max(-1.0, min(1.0, right))
        self._drive_one(self.left, left)
        self._drive_one(self.right, right)

    @staticmethod
    def _drive_one(motor, speed: float) -> None:
        if speed > 0:
            motor.forward(speed)
        elif speed < 0:
            motor.backward(-speed)
        else:
            motor.stop()

    def stop(self) -> None:
        self.left.stop()
        self.right.stop()

    def close(self) -> None:
        self.stop()
        self.left.close()
        self.right.close()
