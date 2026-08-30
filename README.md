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

## Running it

### 1. Clone

```bash
git clone https://github.com/Subhaaaash/mini_bots.git
cd mini_bots
```

### 2. Install

On a dev machine -- no Pi hardware needed, nothing moves:

```bash
python3 -m venv venv
source venv/bin/activate
pip install -r requirements.txt
```

On the Raspberry Pi itself (adds `picamera2`/`gpiozero`, which generally
aren't pip-installable off-Pi):

```bash
python3 -m venv --system-site-packages venv   # --system-site-packages sees apt-installed picamera2
source venv/bin/activate
pip install -r requirements-pi.txt
```

If `picamera2` fails to build via pip, install it from apt instead and make
sure the venv above was created with `--system-site-packages` so it can see it:

```bash
sudo apt install -y python3-picamera2 python3-libcamera
```

### 3. Sanity-check with mock hardware first

The default `config/robot.yaml` has `motor_driver.type: mock`, so this is
safe to run immediately -- it uses the real camera (or a mock one, see
below) but only *prints* motor commands, nothing spins:

```bash
python -m robot_core.main --config config/robot.yaml
```

Open `http://<pi-ip>:8000` (or `http://localhost:8000` on a dev machine) for
the live dashboard: MJPEG stream with the line-detection overlay, telemetry,
and tuning sliders. To run with no camera at all (e.g. on a laptop), set
`camera.type: mock` in `config/robot.yaml` too.

### 4. Switch on real motors

Edit `config/robot.yaml`:

```yaml
motor_driver:
  type: gpiozero_hbridge
  left_pins: [17, 18]   # match your actual wiring
  right_pins: [22, 23]
```

Then rerun the same command as step 3.

### 5. Drive it

- Motors start **stopped** (`control_enabled` defaults to `False`) -- safe.
- Press **START** on the dashboard to let the active behavior actually drive.
- Press **STOP** at any time to force-stop, immediately.
- Arrow keys or the on-screen d-pad give manual (RC-style) control, which
  overrides auto-drive and auto-expires if input stops -- so closing the
  dashboard tab stops the robot rather than leaving it driving blind.
- Sliders (`kp`, `ki`, `kd`, `base_speed`, `max_correction`, ...) tune the
  behavior live, no restart needed.
- `Ctrl+C` in the terminal stops the loop and releases the motors cleanly.

No code changes are required for any of the above -- everything hardware-
or tuning-related lives in `config/robot.yaml`.

## Testing

```bash
# full suite -- kinematics, behavior, safety governor. No hardware required, <1s.
python -m pytest tests/ -q

# a single file, or a single test
python -m pytest tests/test_governor.py -q
python -m pytest tests/test_governor.py::test_lost_target_holds_briefly_then_force_stops -q
```

Every test runs against the mock camera/motor driver, so it passes
identically on a laptop, in CI, or on the Pi.

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
