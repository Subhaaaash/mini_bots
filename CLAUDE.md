# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## What this is

A small open-source, differential-drive robot (Raspberry Pi 3B + Pi Camera
Module 1 + N20 gearmotors) built around a swappable pipeline rather than one
monolithic script, so vision algorithms, motor drivers, and sensors can be
added independently. It ships with a working line follower.

## Commands

```bash
# Install (core deps only -- no Pi hardware needed)
pip install -r requirements.txt

# On the Raspberry Pi itself (adds picamera2/gpiozero, which are ~impossible
# to pip-install off-Pi)
pip install -r requirements-pi.txt

# Run the whole test suite (no hardware required, runs in <1s)
python3 -m pytest tests/ -q

# Run a single test file / test
python3 -m pytest tests/test_governor.py -q
python3 -m pytest tests/test_governor.py::test_lost_target_holds_briefly_then_force_stops -q

# Run the robot (defaults to config/robot.yaml)
python -m robot_core.main --config config/robot.yaml
```

There's no lint/build step configured. `config/robot.yaml`'s `motor_driver.type`
and `camera.type` default to `mock`/hardware as appropriate -- set both to
`mock` to run the full control loop + dashboard on a dev machine with zero
Pi-only imports touched (this is also exactly what the test suite and CI
would exercise).

The original, pre-refactor single-file implementation still lives at
`examples/legacy_single_file.py` and runs standalone (`python
examples/legacy_single_file.py`) -- it has no dependency on `robot_core` and
is kept only as a reference/comparison point, not for further development.

## Architecture

```
CameraSource --frame--> Behavior --DriveCommand(linear,angular)--> SafetyGovernor --wheels--> MotorDriver
                              |                                          ^
                              +---- telemetry dict / debug frame --------+----> Dashboard (Flask + MJPEG)
```

One control-loop iteration (`robot_core/main.py`):
1. `camera.read()` -> a BGR frame.
2. `behavior.compute(frame, dt)` -> `BehaviorResult(command, telemetry, debug_frame)`.
   `command` is `None` when the behavior has nothing to act on (e.g. line lost).
3. `governor.step(result, now)` -> a `GovernorResult` telling the loop
   whether to drive a manual override, drive the behavior's command, force
   a stop, or hold the last command untouched.
4. If there's a command, `kinematics.to_wheel_speeds(command)` converts it
   to `(left, right)` and `motor.drive(left, right)` sends it to hardware.
5. Telemetry + debug frame go to the dashboard, if enabled.

Full design rationale (why DriveCommand is abstract, why safety lives in its
own layer, why hardware imports are deferred) is in `docs/architecture.md` --
read it before making cross-cutting changes to the pipeline. Guides for
adding new pieces: `docs/adding_a_behavior.md`, `docs/adding_a_motor_driver.md`.

### The four extension points, and how they're wired

Each is a small ABC + a plain-dict registry in that package's `__init__.py`,
selected by name from `config/robot.yaml` (built in `robot_core/config.py`
via `_build_from_registry`, which only passes through the config keys a
class's `__init__` actually declares -- so one YAML section can carry keys
for several possible `type`s without the unused ones causing errors):

- `robot_core/behaviors/base.py` (`Behavior`) -- vision/decision algorithms.
  `robot_core/behaviors/line_follow_centerline.py` is the only one so far:
  per-row-midpoint line detection + PID, ported unchanged from the original
  script's `detect_centerline()`.
- `robot_core/motors/base.py` (`MotorDriver`) -- `GpiozeroHBridgeDriver`
  (real hardware) and `MockDriver` (prints commands, replaces the old
  `DRY_RUN` flag; also what `tests/` and dev-off-Pi use).
- `robot_core/camera/base.py` (`CameraSource`) -- `Picamera2Source` and
  `MockCameraSource` (reads a video file/image folder, or synthesizes blank
  frames; **paces itself to `fps` config** so it behaves like a real camera
  timing-wise -- without this the control loop spins unbounded and starves
  the dashboard's Flask thread of GIL time).
- Chassis kinematics -- `robot_core/drive_command.py`'s
  `DifferentialDriveKinematics` is the only implementation; not
  registry-based since there's currently only one chassis shape.

`picamera2`/`gpiozero` imports are deferred inside `Picamera2Source.start()`
and `GpiozeroHBridgeDriver.__init__()` (never at module top level) so the
rest of `robot_core` imports cleanly without those Pi-only packages
installed. Follow this pattern for any new hardware-backed driver/source.

### DriveCommand sign convention

`DriveCommand(linear, angular)` uses the standard CCW-positive convention:
positive `angular` = turn left = **right** wheel spins faster than left
(`DifferentialDriveKinematics.to_wheel_speeds`). Behaviors never emit raw
wheel speeds directly -- this is what keeps them chassis-agnostic.

### SafetyGovernor (`robot_core/safety/governor.py`)

Owns start/stop, manual (RC-style) override with a timeout (losing the
dashboard connection stops the robot instead of leaving it driving), and
lost-target handling (hold briefly, then force-stop after
`max_lost_frames`). Every `Behavior` gets this watchdog for free instead of
reimplementing it. It also tells the main loop when to call
`behavior.reset()` -- any time the behavior isn't the one actively driving,
so PID-style integrators can't wind up while something else has control.

Two deliberate behavior changes vs. the original script, made while
extracting this: motors now force-stop on STOP even if the target happens
to be lost that frame (old code only stopped-on-disable in the
target-found branch), and the behavior resets whenever it's not actively
auto-driving, not only during manual frames.

### Threading

The dashboard (`robot_core/dashboard/app.py`) runs Flask in a second thread
against the same `Behavior`/`SafetyGovernor` instances the control loop
uses. Anything both threads touch is lock-guarded: `SafetyGovernor`'s
internal lock, and `LineFollowCenterline._params_lock` for its tunable
params dict. Follow this pattern for any new behavior that exposes
`get_params`/`set_params`.

### Dashboard sliders are generic

The dashboard doesn't hardcode which params exist -- it fetches
`behavior.param_specs()` (`{name: (min, max, step)}`) and
`behavior.get_params()`/`set_params()` over HTTP (`/param_specs`,
`/params`) and builds sliders from that. A new `Behavior` gets working
dashboard tuning for free just by implementing those three methods; no
dashboard changes needed.
