"""Registry of available Behavior implementations, selected by name from
robot.yaml (`behavior: {type: ...}`). Add a new behavior by writing a
Behavior subclass and registering it here -- see docs/adding_a_behavior.md.
"""

from robot_core.behaviors.base import Behavior, BehaviorResult
from robot_core.behaviors.line_follow_centerline import LineFollowCenterline

BEHAVIOR_REGISTRY = {
    "line_follow_centerline": LineFollowCenterline,
}

__all__ = ["Behavior", "BehaviorResult", "BEHAVIOR_REGISTRY"]
