from robot_core.behaviors.base import BehaviorResult
from robot_core.drive_command import DriveCommand
from robot_core.safety.governor import SafetyGovernor

FOUND = BehaviorResult(command=DriveCommand(linear=0.3, angular=0.1))
LOST = BehaviorResult(command=None)


def test_stopped_by_default_even_when_target_found():
    gov = SafetyGovernor()
    result = gov.step(FOUND, now=0.0)
    assert result.mode == "stopped"
    assert result.should_stop is True


def test_start_enables_auto_driving():
    gov = SafetyGovernor()
    gov.start()
    result = gov.step(FOUND, now=0.0)
    assert result.mode == "auto"
    assert result.command == FOUND.command
    assert result.should_stop is False


def test_lost_target_holds_briefly_then_force_stops():
    gov = SafetyGovernor(max_lost_frames=3)
    gov.start()
    r1 = gov.step(LOST, now=0.0)
    r2 = gov.step(LOST, now=0.03)
    r3 = gov.step(LOST, now=0.06)
    assert r1.mode == "holding" and not r1.should_stop
    assert r2.mode == "holding" and not r2.should_stop
    assert r3.mode == "stopped" and r3.should_stop
    assert r3.lost_streak == 3


def test_found_target_resets_lost_streak():
    gov = SafetyGovernor(max_lost_frames=3)
    gov.start()
    gov.step(LOST, now=0.0)
    gov.step(LOST, now=0.03)
    result = gov.step(FOUND, now=0.06)
    assert result.mode == "auto"
    assert result.lost_streak == 0


def test_stop_forces_stop_even_when_line_lost():
    # Deliberate tightening vs. the original: STOP should immediately halt
    # motors regardless of whether the target happens to be lost that frame.
    gov = SafetyGovernor()
    gov.start()
    gov.stop()
    result = gov.step(LOST, now=0.0)
    assert result.mode == "stopped"
    assert result.should_stop is True


def test_manual_overrides_auto_and_expires():
    gov = SafetyGovernor(manual_timeout=0.4)
    gov.start()
    gov.set_manual(0.5, -0.5, now=10.0)
    result = gov.step(FOUND, now=10.1)
    assert result.mode == "manual"
    assert result.wheel_override == (0.5, -0.5)
    assert result.should_reset_behavior is True
    # control_enabled was cleared by set_manual -- auto doesn't silently
    # resume once the manual window lapses, caller must start() again.
    expired = gov.step(FOUND, now=10.6)
    assert expired.mode == "stopped"


def test_auto_after_manual_is_flagged_for_behavior_reset():
    gov = SafetyGovernor(manual_timeout=0.1)
    gov.set_manual(0.2, 0.2, now=0.0)
    gov.step(FOUND, now=0.05)  # still manual
    gov.start()
    result = gov.step(FOUND, now=0.2)  # manual window lapsed, auto resumes
    assert result.mode == "auto"
    assert result.should_reset_behavior is False
