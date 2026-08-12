# Architecture

```
CameraSource --frame--> Behavior --DriveCommand(linear,angular)--> SafetyGovernor --wheels--> MotorDriver
                              |                                          ^
                              +---- telemetry dict / debug frame --------+----> Dashboard (Flask + MJPEG)
```

One control-loop iteration (`robot_core/main.py`):

1. `camera.read()` -> a BGR frame.
2. `behavior.compute(frame, dt)` -> a `BehaviorResult(command, telemetry, debug_frame)`.
   `command` is `None` if the behavior has nothing to act on this frame
   (e.g. the line/target wasn't found).
3. `governor.step(result, now)` -> a `GovernorResult` telling the loop what
   to actually do: drive a manual override, drive the behavior's command,
   force a stop, or hold the last command.
4. If there's a `DriveCommand`, `kinematics.to_wheel_speeds(command)`
   converts it to `(left, right)` wheel speeds; `motor.drive(left, right)`
   sends them to hardware.
5. Telemetry + the debug frame are pushed to the dashboard, if enabled.

## Why an abstract DriveCommand instead of raw wheel speeds

A `Behavior` never touches wheels directly. It emits
`DriveCommand(linear, angular)`, both roughly in `[-1, 1]`:

- `linear` -- forward/backward speed.
- `angular` -- turn rate, **positive = turn left (CCW)**, which for a
  differential-drive robot means the *right* wheel spins faster than the
  left (`DifferentialDriveKinematics.to_wheel_speeds`, in
  `robot_core/drive_command.py`).

This keeps behaviors chassis-agnostic: a mecanum, mapping a different
kinematics class onto the same `DriveCommand`, would run the exact same
`LineFollowCenterline` unmodified. Motor deadband compensation
(`min_effective_speed`, config `chassis.min_effective_speed`) also lives
here, not in the behavior, since it's a property of the motors, not the
vision algorithm.

## Why a SafetyGovernor instead of each behavior handling safety

Start/stop, manual (RC-style) override with a timeout, and lost-target
handling (hold briefly, then force-stop) used to be inlined in the old
script's frame loop -- meaning a new behavior would have to reimplement all
of it to be safe to run on real hardware. Pulling it into
`robot_core/safety/governor.py` means every behavior gets the same watchdog
for free; a behavior only needs to decide *what it wants the robot to do*,
never *whether it's currently safe to act on that*.

`SafetyGovernor.step()` also tells the loop when to call `behavior.reset()`
(any time the behavior isn't the one actively driving -- manual override or
stopped) so PID-style internal state like an integral term can't wind up
while something else has control.

## Why hardware imports are deferred

`picamera2` and `gpiozero` are only installable (practically) on a
Raspberry Pi with the camera/GPIO stack set up. Their imports live inside
`Picamera2Source.start()` / `GpiozeroHBridgeDriver.__init__()`, not at
module top level, so importing `robot_core` -- for config loading, unit
tests, or running a behavior against `MockCameraSource`/`MockDriver` -- works
on any machine. `requirements.txt` mirrors this split from
`requirements-pi.txt`.

## Extension points

- New vision/decision algorithm -> implement `Behavior`
  (`robot_core/behaviors/base.py`). See `adding_a_behavior.md`.
- New motor backend (different H-bridge, I2C motor HAT, ...) -> implement
  `MotorDriver` (`robot_core/motors/base.py`). See `adding_a_motor_driver.md`.
- New camera/frame source -> implement `CameraSource`
  (`robot_core/camera/base.py`), same pattern as `MotorDriver`.
- New chassis kinematics (mecanum, ackermann, ...) -> implement a class with
  a `to_wheel_speeds(DriveCommand)`-shaped method (or more, for >2 wheels)
  alongside `DifferentialDriveKinematics`.

Each of these is selected by name from `config/robot.yaml` via a small
registry dict in the corresponding package's `__init__.py` -- no code
changes needed to switch between existing implementations, and adding a new
one is "write the class + add one line to the registry."

## Not done (on purpose)

- **No plugin/entry-point discovery yet.** Registries are plain dicts; a new
  behavior/driver requires adding it to this repo's registry rather than
  being auto-discovered from a separately `pip install`ed package. Worth
  revisiting once there are third-party behaviors to support.
- **Single process, in-process calls.** `Behavior` -> `SafetyGovernor` ->
  `MotorDriver` are plain Python function calls on one thread; the
  dashboard runs on a second thread (Flask) talking to the same objects
  through small locks (see `SafetyGovernor`'s internal lock and
  `LineFollowCenterline`'s `_params_lock`). The data flowing between stages
  (`DriveCommand`, `BehaviorResult`, telemetry dicts) is deliberately
  simple/serializable, so splitting this across processes or machines later
  (e.g. heavier vision on a laptop, a lightweight motor-control process on
  the Pi) wouldn't require redesigning the interfaces -- just adding a
  transport underneath them.
