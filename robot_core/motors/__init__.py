"""Registry of available MotorDriver implementations, selected by name from
robot.yaml (`motor_driver: {type: ...}`). Add a new driver by writing a
MotorDriver subclass and registering it here -- see docs/adding_a_motor_driver.md.
"""

from robot_core.motors.base import MotorDriver
from robot_core.motors.mock import MockDriver

MOTOR_DRIVER_REGISTRY = {
    "mock": MockDriver,
}

try:
    from robot_core.motors.gpiozero_hbridge import GpiozeroHBridgeDriver

    MOTOR_DRIVER_REGISTRY["gpiozero_hbridge"] = GpiozeroHBridgeDriver
except ImportError:
    # gpiozero_hbridge.py itself has no top-level hardware imports, so this
    # only fails if robot_core.motors.base or similar is broken -- kept as a
    # defensive guard, not the primary way Pi-only code is isolated.
    pass

__all__ = ["MotorDriver", "MOTOR_DRIVER_REGISTRY"]
