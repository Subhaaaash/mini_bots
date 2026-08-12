import numpy as np

from robot_core.behaviors.line_follow_centerline import LineFollowCenterline


def make_frame_with_line(width=640, height=480, line_x_frac=0.5, line_width=20):
    """A white frame with a straight vertical black line at the given
    horizontal fraction of the width, used to drive detect_centerline()."""
    frame = np.full((height, width, 3), 255, dtype=np.uint8)
    center_x = int(width * line_x_frac)
    x0 = max(0, center_x - line_width // 2)
    x1 = min(width, center_x + line_width // 2)
    frame[:, x0:x1] = (0, 0, 0)
    return frame


def test_line_centered_gives_near_zero_command():
    behavior = LineFollowCenterline(draw_debug=False)
    frame = make_frame_with_line(line_x_frac=0.5)
    result = behavior.compute(frame, dt=1 / 30)
    assert result.command is not None
    assert abs(result.command.angular) < 0.05
    assert result.command.linear > 0


def test_line_offset_right_turns_right():
    # Line to the right of center -> robot should turn right, i.e. negative
    # angular under the CCW-positive convention.
    behavior = LineFollowCenterline(draw_debug=False)
    frame = make_frame_with_line(line_x_frac=0.75)
    result = behavior.compute(frame, dt=1 / 30)
    assert result.command is not None
    assert result.command.angular < 0


def test_line_offset_left_turns_left():
    behavior = LineFollowCenterline(draw_debug=False)
    frame = make_frame_with_line(line_x_frac=0.25)
    result = behavior.compute(frame, dt=1 / 30)
    assert result.command is not None
    assert result.command.angular > 0


def test_no_line_returns_none_command():
    behavior = LineFollowCenterline(draw_debug=False)
    frame = np.full((480, 640, 3), 255, dtype=np.uint8)  # all white, no line
    result = behavior.compute(frame, dt=1 / 30)
    assert result.command is None
    assert result.telemetry["near_error"] is None


def test_reset_clears_pid_state():
    behavior = LineFollowCenterline(draw_debug=False)
    frame = make_frame_with_line(line_x_frac=0.75)
    behavior.compute(frame, dt=1 / 30)
    assert behavior._integral != 0.0
    behavior.reset()
    assert behavior._integral == 0.0
    assert behavior._prev_error == 0.0


def test_param_specs_and_set_params_roundtrip():
    behavior = LineFollowCenterline(draw_debug=False)
    specs = behavior.param_specs()
    assert "kp" in specs
    behavior.set_params({"kp": 0.01})
    assert behavior.get_params()["kp"] == 0.01
