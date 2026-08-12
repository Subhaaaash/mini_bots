import math

from robot_core.drive_command import DriveCommand, DifferentialDriveKinematics


def test_straight_forward():
    k = DifferentialDriveKinematics()
    left, right = k.to_wheel_speeds(DriveCommand(linear=0.5, angular=0.0))
    assert left == right == 0.5


def test_turn_left_speeds_up_right_wheel():
    k = DifferentialDriveKinematics()
    left, right = k.to_wheel_speeds(DriveCommand(linear=0.3, angular=0.2))
    assert right > left
    assert math.isclose(right - left, 0.4)


def test_clamps_combined_speed_to_unit_range():
    k = DifferentialDriveKinematics()
    left, right = k.to_wheel_speeds(DriveCommand(linear=0.9, angular=0.9))
    assert -1.0 <= left <= 1.0
    assert -1.0 <= right <= 1.0
    # ratio between wheels should still reflect the requested turn
    assert right > left


def test_zero_command_is_stopped():
    k = DifferentialDriveKinematics()
    left, right = k.to_wheel_speeds(DriveCommand())
    assert left == 0.0
    assert right == 0.0


def test_min_effective_speed_boosts_small_commands_preserving_ratio():
    k = DifferentialDriveKinematics(min_effective_speed=0.25)
    left, right = k.to_wheel_speeds(DriveCommand(linear=0.1, angular=0.0))
    assert left == right
    assert abs(left) >= 0.25 - 1e-9
