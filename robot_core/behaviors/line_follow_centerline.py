"""LineFollowCenterline -- the original vision algorithm, unchanged.

Detects a single dark line via per-row midpoint (centerline) detection in a
region of interest, fits a line through the midpoints to get a near-field
lateral error and heading angle, and runs those through a PID loop to
produce a DriveCommand. This is a straight port of the detect_centerline()
function and PID math from the original single-file script -- only the
packaging changed (module-level constants -> instance params, module-level
PID state -> instance attributes reset via reset()).
"""

import math
import threading
from typing import Any

import cv2
import numpy as np

from robot_core.behaviors.base import Behavior, BehaviorResult
from robot_core.drive_command import DriveCommand

# ---- Threshold / morphology (validated against the physical track) ----
BLACK_LOW = (0, 0, 0)
BLACK_HIGH = (100, 100, 100)
ERODE_ITERATIONS = 2
DILATE_ITERATIONS = 6
MIN_ROW_WIDTH_PX = 3

PARAM_SPECS = {
    "kp": (0.0, 0.02, 0.0002),
    "ki": (0.0, 0.002, 0.00005),
    "kd": (0.0, 0.01, 0.0002),
    "base_speed": (0.0, 0.8, 0.01),
    "max_correction": (0.0, 0.6, 0.01),
    "heading_weight": (0.0, 3.0, 0.05),
    "lookahead_px": (0.0, 400.0, 5.0),
}

DEFAULT_PARAMS = {
    "kp": 0.003,
    "ki": 0.0001,
    "kd": 0.001,
    "base_speed": 0.35,
    "max_correction": 0.3,
    "heading_weight": 1.0,
    "lookahead_px": 150.0,
}

INTEGRAL_CLAMP = 200.0


def detect_centerline(frame_bgr: np.ndarray, roi_top_frac: float, roi_bottom_frac: float, draw_debug: bool = False):
    """Returns (near_error, slope, angle_deg, num_points, debug_frame_or_None).
    near_error/slope/angle_deg are None if detection failed.
    """
    h, w = frame_bgr.shape[:2]
    mid = w // 2

    roi_top = int(h * roi_top_frac)
    roi_bottom = int(h * roi_bottom_frac)
    roi = frame_bgr[roi_top:roi_bottom, :]

    blackline = cv2.inRange(roi, BLACK_LOW, BLACK_HIGH)
    kernel = np.ones((3, 3), np.uint8)
    blackline = cv2.erode(blackline, kernel, iterations=ERODE_ITERATIONS)
    blackline = cv2.dilate(blackline, kernel, iterations=DILATE_ITERATIONS)

    contours, _ = cv2.findContours(blackline.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)

    debug = None
    if draw_debug:
        debug = frame_bgr.copy()
        cv2.rectangle(debug, (0, roi_top), (w, roi_bottom), (0, 255, 255), 1)
        cv2.line(debug, (mid, 0), (mid, h), (0, 0, 255), 1)

    if len(contours) == 0:
        if draw_debug:
            cv2.putText(debug, "NO CONTOUR", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255), 2)
        return None, None, None, 0, debug

    largest = max(contours, key=cv2.contourArea)
    if draw_debug:
        outline_full = largest + np.array([0, roi_top])
        cv2.drawContours(debug, [outline_full], -1, (0, 165, 255), 2)

    single_mask = np.zeros_like(blackline)
    cv2.drawContours(single_mask, [largest], -1, 255, thickness=cv2.FILLED)

    roi_h = single_mask.shape[0]
    centerline_pts = []

    for y in range(roi_h):
        row = single_mask[y, :]
        xs = np.where(row > 0)[0]
        if len(xs) < MIN_ROW_WIDTH_PX:
            continue
        left_x = xs[0]
        right_x = xs[-1]
        mid_x = (left_x + right_x) / 2.0
        y_full = y + roi_top
        centerline_pts.append((y_full, mid_x))
        if draw_debug:
            cv2.circle(debug, (int(mid_x), y_full), 1, (255, 0, 0), -1)

    if len(centerline_pts) < 2:
        if draw_debug:
            cv2.putText(debug, "NOT ENOUGH POINTS", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255), 2)
        return None, None, None, len(centerline_pts), debug

    ys = np.array([p[0] for p in centerline_pts], dtype=np.float32)
    xs = np.array([p[1] for p in centerline_pts], dtype=np.float32)
    m, b = np.polyfit(ys, xs, 1)

    y_eval = roi_bottom
    near_x = m * y_eval + b
    near_error = near_x - mid
    angle_deg = math.degrees(math.atan(m))

    if draw_debug:
        y_top, y_bot = int(ys.min()), int(ys.max())
        x_top, x_bot = int(m * y_top + b), int(m * y_bot + b)
        cv2.line(debug, (x_top, y_top), (x_bot, y_bot), (0, 0, 255), 3)
        cv2.circle(debug, (int(near_x), y_eval), 6, (0, 255, 0), -1)
        cv2.putText(debug, f"angle={angle_deg:+.1f}deg", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
        cv2.putText(debug, f"near_err={near_error:+.1f}px", (10, 58), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 0, 0), 2)

    return near_error, m, angle_deg, len(centerline_pts), debug


class LineFollowCenterline(Behavior):
    def __init__(
        self,
        roi_top_frac: float = 0.50,
        roi_bottom_frac: float = 0.85,
        draw_debug: bool = True,
        **params: Any,
    ):
        self.roi_top_frac = roi_top_frac
        self.roi_bottom_frac = roi_bottom_frac
        self.draw_debug = draw_debug
        self.params = {**DEFAULT_PARAMS, **params}
        self._params_lock = threading.Lock()  # params are read on the control
        # loop thread and written from the dashboard's Flask thread
        self._integral = 0.0
        self._prev_error = 0.0

    def reset(self) -> None:
        self._integral = 0.0
        self._prev_error = 0.0

    def compute(self, frame_bgr: np.ndarray, dt: float) -> BehaviorResult:
        near_error, slope, angle_deg, num_pts, debug = detect_centerline(
            frame_bgr, self.roi_top_frac, self.roi_bottom_frac, draw_debug=self.draw_debug
        )

        telemetry = {
            "near_error": float(near_error) if near_error is not None else None,
            "angle_deg": float(angle_deg) if angle_deg is not None else None,
            "num_points": num_pts,
        }

        if near_error is None:
            return BehaviorResult(command=None, telemetry=telemetry, debug_frame=debug)

        with self._params_lock:
            p = dict(self.params)
        combined_error = near_error + p["heading_weight"] * slope * p["lookahead_px"]

        self._integral += combined_error * dt
        self._integral = max(-INTEGRAL_CLAMP, min(INTEGRAL_CLAMP, self._integral))
        derivative = (combined_error - self._prev_error) / dt if dt > 0 else 0.0
        self._prev_error = combined_error

        correction = p["kp"] * combined_error + p["ki"] * self._integral + p["kd"] * derivative
        correction = max(-p["max_correction"], min(p["max_correction"], correction))

        # Positive combined_error means the line is to the right of center,
        # i.e. the robot needs to turn right, which is negative angular
        # under the CCW-positive convention used by DriveCommand.
        command = DriveCommand(linear=p["base_speed"], angular=-correction)

        telemetry["combined_error"] = float(combined_error)
        telemetry["correction"] = float(correction)

        if debug is not None:
            cv2.putText(debug, f"correction={correction:+.3f}", (10, 86),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 200, 0), 2)

        return BehaviorResult(command=command, telemetry=telemetry, debug_frame=debug)

    def param_specs(self) -> dict[str, tuple[float, float, float]]:
        return dict(PARAM_SPECS)

    def get_params(self) -> dict[str, Any]:
        with self._params_lock:
            return dict(self.params)

    def set_params(self, params: dict[str, Any]) -> None:
        with self._params_lock:
            for k, v in params.items():
                if k in self.params:
                    self.params[k] = float(v)
