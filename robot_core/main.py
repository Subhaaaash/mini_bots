"""Entrypoint: wires CameraSource -> Behavior -> SafetyGovernor -> MotorDriver
(+ optional dashboard) together from a YAML config and runs the control
loop. This replaces the old script's monolithic main() -- the loop itself
is now ~30 lines because every piece of logic lives in its own module.

Usage:
  python -m robot_core.main --config config/robot.yaml
"""

import argparse
import time

import cv2

from robot_core.config import (
    build_behavior,
    build_camera,
    build_governor,
    build_kinematics,
    build_motor_driver,
    load_config,
)
from robot_core.dashboard.app import DashboardApp


def run(config: dict) -> None:
    camera = build_camera(config["camera"])
    motor = build_motor_driver(config["motor_driver"])
    behavior = build_behavior(config["behavior"])
    kinematics = build_kinematics(config.get("chassis", {}))
    governor = build_governor(config.get("safety", {}))

    dash_cfg = config.get("dashboard", {})
    dashboard = None
    if dash_cfg.get("enabled", True):
        dashboard = DashboardApp(behavior, governor, title=config["behavior"]["type"])
        port = dash_cfg.get("port", 8000)
        dashboard.run_in_thread(host=dash_cfg.get("host", "0.0.0.0"), port=port)
        print(f"Dashboard: http://<robot-ip>:{port}")
    jpeg_quality = dash_cfg.get("jpeg_quality", 70)

    camera.start()
    time.sleep(1)  # let auto-exposure/auto-white-balance settle

    print(f"Starting behavior={config['behavior']['type']!r} "
          f"motor_driver={config['motor_driver']['type']!r} "
          f"camera={config['camera']['type']!r}. Ctrl+C to stop.\n")

    prev_time = time.time()
    frame_count = 0
    t_start = prev_time

    try:
        while True:
            frame = camera.read()
            now = time.time()
            dt = now - prev_time
            prev_time = now

            result = behavior.compute(frame, dt)
            gov_result = governor.step(result, now)

            if gov_result.should_reset_behavior:
                behavior.reset()

            if gov_result.wheel_override is not None:
                left, right = gov_result.wheel_override
                motor.drive(left, right)
            elif gov_result.should_stop:
                motor.stop()
            elif gov_result.command is not None:
                left, right = kinematics.to_wheel_speeds(gov_result.command)
                motor.drive(left, right)
            # else ("holding"): leave the motors at their last commanded
            # speed rather than jerking to a stop on one dropped frame.

            frame_count += 1
            if frame_count % 5 == 0:
                extra = " ".join(
                    f"{k}={v:+.2f}" if isinstance(v, float) else f"{k}={v}"
                    for k, v in result.telemetry.items() if v is not None
                )
                print(f"[{gov_result.mode}] lost_streak={gov_result.lost_streak} {extra}")

            if dashboard is not None:
                status = {"mode": gov_result.mode, "lost_streak": gov_result.lost_streak, **result.telemetry}
                dashboard.update_status(status)
                debug_frame = result.debug_frame if result.debug_frame is not None else frame
                ok, jpeg = cv2.imencode(".jpg", debug_frame, [cv2.IMWRITE_JPEG_QUALITY, jpeg_quality])
                if ok:
                    dashboard.update_frame(jpeg.tobytes())

    except KeyboardInterrupt:
        elapsed = time.time() - t_start
        fps = frame_count / elapsed if elapsed > 0 else 0.0
        print(f"\nStopped by user. {frame_count} frames in {elapsed:.1f}s ({fps:.1f} fps avg).")
    finally:
        motor.stop()
        motor.close()
        camera.stop()


def main() -> None:
    parser = argparse.ArgumentParser(description="Run the robot control loop from a YAML config.")
    parser.add_argument("--config", default="config/robot.yaml", help="Path to a robot.yaml config file")
    args = parser.parse_args()
    run(load_config(args.config))


if __name__ == "__main__":
    main()
