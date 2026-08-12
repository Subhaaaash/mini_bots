# Adding a Motor Driver

A `MotorDriver` takes two normalized wheel speeds in `[-1, 1]` and moves the
robot. Implement this to support a different H-bridge, an I2C motor HAT
(e.g. a PCA9685), a serial-connected motor controller, or anything else.

## 1. Implement the interface

Create `robot_core/motors/my_driver.py`:

```python
from robot_core.motors.base import MotorDriver


class MyDriver(MotorDriver):
    def __init__(self, i2c_address: int = 0x40):
        # Defer hardware-library imports into __init__ (not module top
        # level) if the library only installs on the target board -- this
        # keeps the rest of robot_core importable on a dev laptop. See
        # robot_core/motors/gpiozero_hbridge.py for the pattern.
        from my_motor_hat_library import MotorHat  # example

        self._hat = MotorHat(address=i2c_address)

    def drive(self, left: float, right: float) -> None:
        left = max(-1.0, min(1.0, left))
        right = max(-1.0, min(1.0, right))
        self._hat.set_left(left)
        self._hat.set_right(right)

    def stop(self) -> None:
        self.drive(0.0, 0.0)

    def close(self) -> None:
        self.stop()
        self._hat.release()
```

`close()` has a default implementation (`self.stop()`); override it if you
need to release hardware resources too.

## 2. Register it

In `robot_core/motors/__init__.py`:

```python
from robot_core.motors.my_driver import MyDriver

MOTOR_DRIVER_REGISTRY = {
    "mock": MockDriver,
    "gpiozero_hbridge": GpiozeroHBridgeDriver,
    "my_driver": MyDriver,
}
```

## 3. Select it in config

```yaml
motor_driver:
  type: my_driver
  i2c_address: 66
```

Only the keys your `__init__` actually accepts are passed through (see
`robot_core/config.py::_build_from_registry`), so it's fine for
`motor_driver:` in `robot.yaml` to also carry keys other driver types use
(e.g. `left_pins`/`right_pins` for `gpiozero_hbridge`) -- they're just
ignored.

## 4. Test it without hardware

Use `MockDriver` (`robot_core/motors/mock.py`) as the reference: it
implements the same interface, records the last commanded speeds, and
prints them -- that's what `motor_driver: {type: mock}` in `robot.yaml`
gives you (this replaces the old script's `DRY_RUN = True` flag).
