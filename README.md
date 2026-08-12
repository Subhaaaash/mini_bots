# tiny_swarm_n20_robot

A small, open-source, differential-drive robot built around a Raspberry Pi,
a single Pi Camera, and N20 gearmotors. It ships with a working line
follower, but the core is a swappable pipeline so you can drop in your own
vision algorithm, motor driver, or sensor without touching the rest of the
code.

```
CameraSource --frame--> Behavior --DriveCommand(linear, angular)--> SafetyGovernor --wheels--> MotorDriver
                              |                                          ^
                              +---- telemetry / debug frame -------------+----> Dashboard (Flask + MJPEG)
```

- **Behavior** -- the pluggable vision/decision algorithm (line following,
  obstacle avoidance, ArUco following, ...). See
  [`docs/adding_a_behavior.md`](docs/adding_a_behavior.md).
- **MotorDriver** -- the pluggable motor backend (gpiozero H-bridge, a mock
  for testing, or your own). See
  [`docs/adding_a_motor_driver.md`](docs/adding_a_motor_driver.md).
- **SafetyGovernor** -- start/stop, manual override, and lost-target
  watchdog shared by every behavior.
- **Dashboard** -- live MJPEG stream, start/stop, manual d-pad, and
  per-behavior tuning sliders at `http://<pi-ip>:8000`.

Full design notes: [`docs/architecture.md`](docs/architecture.md).

## Quick start

```bash
# On a dev machine (no Pi hardware needed) -- runs against a mock camera/motors
pip install -r requirements.txt
python -m robot_core.main --config config/robot.yaml

# On the Raspberry Pi
pip install -r requirements-pi.txt
python -m robot_core.main --config config/robot.yaml
```

Edit `config/robot.yaml` to pick which behavior/camera/motor driver to run
and to tune parameters -- no code changes required for common tweaks.

## Repo layout

```
robot_core/       core library: drive_command, behaviors, motors, camera, safety, dashboard, config, main
config/           robot.yaml -- hardware + behavior selection and tuning
docs/             architecture + "how to add a X" guides
tests/            unit tests (run without any Pi hardware)
examples/         standalone example scripts, including the original single-file line follower
```

## Status

The original single-file line follower lives at
[`examples/legacy_single_file.py`](examples/legacy_single_file.py) and still
runs standalone -- useful as a reference while the modular version is new.

## Contributing

New behaviors, motor drivers, and sensor add-ons are welcome -- that's the
whole point of the split. Start with
[`docs/adding_a_behavior.md`](docs/adding_a_behavior.md) or
[`docs/adding_a_motor_driver.md`](docs/adding_a_motor_driver.md).
