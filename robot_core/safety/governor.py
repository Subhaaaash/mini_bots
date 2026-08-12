"""SafetyGovernor -- the watchdog every Behavior gets for free.

Sits between a Behavior's output and the MotorDriver. Owns:
  - start/stop (control_enabled) -- motors only move when explicitly started
  - manual (RC-style) override -- takes over from auto, expires on its own
    if not refreshed (losing the dashboard connection stops the robot
    instead of leaving it stuck driving)
  - lost-target handling -- briefly holds the last command, then forces a
    stop if the target stays lost too long

This is a straight port of the control/manual/lost-frame logic that used to
be inlined in the old script's main() loop (module-level control_enabled /
manual_* globals + the if/elif/else in the frame loop), with two small,
deliberate safety tightenings made while separating it out:
  1. Motors are force-stopped whenever control is disabled, even if the
     target is currently lost (old code only stopped-on-disable in the
     "target found" branch, so pressing STOP while the line was lost didn't
     immediately stop the motors).
  2. The behavior is reset (e.g. PID integral cleared) any time it's not
     actively auto-driving (manual or stopped), not only during manual
     frames, so windup can't accumulate while stopped.

Thread-safe: start()/stop()/set_manual() are typically called from a Flask
request thread while step() runs on the control loop thread.
"""

import threading
from dataclasses import dataclass
from typing import Optional

from robot_core.behaviors.base import BehaviorResult
from robot_core.drive_command import DriveCommand


@dataclass
class GovernorResult:
    mode: str  # "manual" | "auto" | "holding" | "stopped"
    should_stop: bool
    should_reset_behavior: bool
    lost_streak: int
    wheel_override: Optional[tuple] = None  # (left, right), manual mode only
    command: Optional[DriveCommand] = None  # auto mode only, still needs kinematics


class SafetyGovernor:
    def __init__(self, max_lost_frames: int = 10, manual_timeout: float = 0.4):
        self.max_lost_frames = max_lost_frames
        self.manual_timeout = manual_timeout

        self._lock = threading.Lock()
        self.control_enabled = False
        self._manual_left = 0.0
        self._manual_right = 0.0
        self._manual_until = 0.0
        self._lost_streak = 0

    def start(self) -> None:
        with self._lock:
            self.control_enabled = True

    def stop(self) -> None:
        with self._lock:
            self.control_enabled = False

    def set_manual(self, left: float, right: float, now: float) -> None:
        with self._lock:
            self._manual_left = left
            self._manual_right = right
            self._manual_until = now + self.manual_timeout
            # Manual input always takes over from auto -- avoid the two
            # fighting for the motors. Caller must start() again to resume.
            self.control_enabled = False

    def step(self, behavior_result: Optional[BehaviorResult], now: float) -> GovernorResult:
        with self._lock:
            manual_active = now < self._manual_until
            manual_left, manual_right = self._manual_left, self._manual_right
            control_enabled = self.control_enabled

        if manual_active:
            self._lost_streak = 0
            return GovernorResult(
                mode="manual", should_stop=False, should_reset_behavior=True,
                lost_streak=0, wheel_override=(manual_left, manual_right),
            )

        command = behavior_result.command if behavior_result else None

        if command is not None:
            self._lost_streak = 0
            if not control_enabled:
                return GovernorResult(mode="stopped", should_stop=True,
                                       should_reset_behavior=True, lost_streak=0)
            return GovernorResult(mode="auto", should_stop=False,
                                   should_reset_behavior=False, lost_streak=0, command=command)

        # Target lost this frame.
        self._lost_streak += 1
        if not control_enabled:
            return GovernorResult(mode="stopped", should_stop=True,
                                   should_reset_behavior=True, lost_streak=self._lost_streak)
        if self._lost_streak >= self.max_lost_frames:
            return GovernorResult(mode="stopped", should_stop=True,
                                   should_reset_behavior=True, lost_streak=self._lost_streak)
        # Briefly hold the last command rather than jerking to a stop on a
        # single dropped frame.
        return GovernorResult(mode="holding", should_stop=False,
                               should_reset_behavior=False, lost_streak=self._lost_streak)
