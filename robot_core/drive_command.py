"""The contract between a Behavior and a MotorDriver.

Behaviors never emit raw wheel speeds. They emit an abstract, chassis-agnostic
DriveCommand(linear, angular), both normalized to roughly [-1, 1]:

  linear  -- forward/backward speed (+1 = full forward, -1 = full reverse)
  angular -- turn rate (+1 = full turn left/CCW, -1 = full turn right/CW)

DifferentialDriveKinematics converts that into per-wheel speeds for a
two-wheel differential-drive chassis. A future mecanum/ackermann/etc. chassis
would get its own kinematics class -- every existing Behavior keeps working
unchanged, since it never knew about wheels in the first place.
"""

from dataclasses import dataclass


@dataclass
class DriveCommand:
    linear: float = 0.0
    angular: float = 0.0

    def clamped(self) -> "DriveCommand":
        return DriveCommand(
            linear=max(-1.0, min(1.0, self.linear)),
            angular=max(-1.0, min(1.0, self.angular)),
        )


class DifferentialDriveKinematics:
    """Converts DriveCommand -> (left_speed, right_speed) for a 2-wheel robot.

    min_effective_speed compensates for real-motor deadband (a motor given a
    tiny PWM duty cycle just hums and doesn't turn): if both wheels are
    commanded but the larger magnitude is below this threshold, both are
    scaled up together so the ratio (and therefore the intended turn) is
    preserved. This mirrors the old apply_min_effective_speed_pair() helper,
    just relocated here since it's a motor-hardware concern, not something a
    vision Behavior should need to know about.
    """

    def __init__(self, min_effective_speed: float = 0.0):
        self.min_effective_speed = min_effective_speed

    def to_wheel_speeds(self, command: DriveCommand) -> tuple[float, float]:
        cmd = command.clamped()
        # Standard CCW-positive convention: turning left means the right
        # wheel spins faster than the left wheel.
        left = cmd.linear - cmd.angular
        right = cmd.linear + cmd.angular

        max_abs = max(abs(left), abs(right))
        if max_abs > 1.0:
            left /= max_abs
            right /= max_abs
        elif 0.0 < max_abs < self.min_effective_speed:
            scale = self.min_effective_speed / max_abs
            left *= scale
            right *= scale

        return left, right
