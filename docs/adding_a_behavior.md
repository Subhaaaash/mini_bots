# Adding a Behavior

A `Behavior` is the pluggable vision/decision block: it looks at one frame
and decides what the robot should do next. It never touches motors or GPIO.

## 1. Implement the interface

Create `robot_core/behaviors/my_behavior.py`:

```python
from robot_core.behaviors.base import Behavior, BehaviorResult
from robot_core.drive_command import DriveCommand


class MyBehavior(Behavior):
    def __init__(self, some_threshold: float = 0.5, **params):
        self.some_threshold = some_threshold
        self.params = {"gain": 0.5, **params}

    def reset(self) -> None:
        # Called whenever this behavior isn't the one actively driving
        # (manual override active, or motors stopped) -- clear any PID
        # integrator / temporal state here so it doesn't wind up.
        pass

    def compute(self, frame_bgr, dt: float) -> BehaviorResult:
        target_found, error = my_detection(frame_bgr, self.some_threshold)
        if not target_found:
            # None means "nothing to act on this frame" -- the
            # SafetyGovernor decides whether to hold or stop, you don't
            # need your own lost-target policy here.
            return BehaviorResult(command=None, telemetry={"found": False})

        command = DriveCommand(linear=0.3, angular=-self.params["gain"] * error)
        return BehaviorResult(command=command, telemetry={"found": True, "error": error})

    # Optional: only needed if you want live-tunable dashboard sliders.
    def param_specs(self):
        return {"gain": (0.0, 2.0, 0.01)}  # name: (min, max, step)

    def get_params(self):
        return dict(self.params)

    def set_params(self, params):
        for k, v in params.items():
            if k in self.params:
                self.params[k] = float(v)
```

Notes:
- `DriveCommand(linear, angular)` is chassis-agnostic -- `linear` is
  forward/backward speed, `angular` is turn rate with **positive = turn
  left**. See `docs/architecture.md` for the sign convention and why it's
  not raw wheel speeds.
- If your params dict is read from the dashboard's Flask thread while
  `compute()` runs on the control-loop thread (true whenever the dashboard
  is enabled), guard it with a lock -- see `LineFollowCenterline` in
  `robot_core/behaviors/line_follow_centerline.py` for the pattern
  (`self._params_lock`).
- `telemetry` can hold anything JSON-able; it shows up on the dashboard
  status line automatically, no dashboard changes needed.

## 2. Register it

In `robot_core/behaviors/__init__.py`:

```python
from robot_core.behaviors.my_behavior import MyBehavior

BEHAVIOR_REGISTRY = {
    "line_follow_centerline": LineFollowCenterline,
    "my_behavior": MyBehavior,
}
```

## 3. Select it in config

```yaml
behavior:
  type: my_behavior
  some_threshold: 0.6
  params:
    gain: 0.8
```

`some_threshold` maps to a constructor argument; anything under `params:`
is passed through as `**params` (so it ends up in `self.params`, tunable
live via the dashboard if you implemented `param_specs`/`get_params`/
`set_params`).

## 4. Test it without hardware

```python
import numpy as np
from robot_core.behaviors.my_behavior import MyBehavior

behavior = MyBehavior()
frame = np.full((480, 640, 3), 255, dtype=np.uint8)  # or a real test image
result = behavior.compute(frame, dt=1/30)
assert result.command is None  # or whatever you expect
```

See `tests/test_line_follow_behavior.py` for a fuller example (synthetic
frames, sign-convention checks, param round-tripping).
