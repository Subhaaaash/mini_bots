"""
LEGACY / REFERENCE ONLY. This is the original single-file implementation,
kept as-is for comparison while the modular robot_core/ version is new. For
the actively developed, pluggable version, run `python -m robot_core.main`
instead -- see the top-level README and docs/architecture.md. This file has
no dependency on robot_core and still runs standalone.

curve_follow_centerline.py
Real-time closed-loop single-line following using the centerline
(per-row midpoint) detection method, with PID control.

DRY_RUN = True   -> prints motor commands but doesn't move
DRY_RUN = False  -> actually drives the motors

ENABLE_STREAM = True  -> serves a live MJPEG debug stream at
                         http://<pi-ip>:8000/stream.mjpg (view with ffplay)
ENABLE_STREAM = False -> no streaming overhead, just runs the control loop

Press Ctrl+C to stop (motors released in finally block).
"""

from picamera2 import Picamera2
from libcamera import Transform
from gpiozero import Motor
from flask import Flask, Response, request, jsonify
import cv2
import numpy as np
import time
import math
import threading

# =========================================================
# SAFETY / DEBUG TOGGLES
# =========================================================
DRY_RUN = True
ENABLE_STREAM = True

# ---- Camera config ----
FRAME_W, FRAME_H = 640, 480
ROI_TOP_FRAC = 0.50
ROI_BOTTOM_FRAC = 0.85

# ---- Threshold / morphology (validated) ----
BLACK_LOW = (0, 0, 0)
BLACK_HIGH = (100, 100, 100)
ERODE_ITERATIONS = 2
DILATE_ITERATIONS = 6
MIN_ROW_WIDTH_PX = 3

# ---- Combined signal tuning ----
LOOKAHEAD_PX = 150
HEADING_WEIGHT = 1.0

# ---- Motor pins ----
LEFT_IN1, LEFT_IN2 = 17, 18
RIGHT_IN3, RIGHT_IN4 = 22, 23

# ---- PID tuning ----
BASE_SPEED = 0.35
KP = 0.003
KI = 0.0001
KD = 0.001
MAX_CORRECTION = 0.3
INTEGRAL_CLAMP = 200.0
MAX_LOST_FRAMES = 10
MIN_EFFECTIVE_SPEED = 0.25

JPEG_QUALITY = 70

# ---- Shared state for streaming (written by control loop, read by Flask thread) ----
stream_lock = threading.Lock()
latest_jpeg = None

# ---- Shared tunable parameters (adjustable live via dashboard sliders) ----
params_lock = threading.Lock()
params = {
    "kp": KP,
    "ki": KI,
    "kd": KD,
    "base_speed": BASE_SPEED,
    "max_correction": MAX_CORRECTION,
    "heading_weight": HEADING_WEIGHT,
    "lookahead_px": LOOKAHEAD_PX,
    "min_effective_speed": MIN_EFFECTIVE_SPEED,
    "manual_speed": 0.4,
}

# ---- Runtime start/stop control (independent of DRY_RUN) ----
# Motors only actually move when control_enabled is True AND DRY_RUN is False.
# Starts stopped for safety -- press Start on the dashboard to enable driving.
control_lock = threading.Lock()
control_enabled = False

# ---- Manual (RC-style) override ----
# Any manual command disables auto (control_enabled) and takes over the
# motors directly. A command "expires" after MANUAL_TIMEOUT seconds if not
# refreshed -- this means losing network/closing the browser stops the
# robot automatically rather than leaving it stuck driving.
manual_lock = threading.Lock()
manual_left = 0.0
manual_right = 0.0
manual_until = 0.0
MANUAL_TIMEOUT = 0.4
was_manual_active = False

# ---- Latest status, for the dashboard readout ----
status_lock = threading.Lock()
status = {"near_error": None, "angle_deg": None, "left_speed": None,
          "right_speed": None, "lost_streak": 0, "mode": "stopped"}


def apply_min_effective_speed_pair(left, right, min_speed):
    max_abs = max(abs(left), abs(right))
    if max_abs == 0:
        return 0.0, 0.0
    if max_abs < min_speed:
        scale = min_speed / max_abs
        left *= scale
        right *= scale
    return left, right


def detect_centerline(frame_bgr, draw_debug=False):
    """
    Returns (near_error, slope, angle_deg, num_points, debug_frame_or_None)
    near_error/slope/angle_deg are None if detection failed.
    """
    h, w = frame_bgr.shape[:2]
    mid = w // 2

    roi_top = int(h * ROI_TOP_FRAC)
    roi_bottom = int(h * ROI_BOTTOM_FRAC)
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


class DiffDrive:
    def __init__(self):
        self.left = Motor(forward=LEFT_IN1, backward=LEFT_IN2)
        self.right = Motor(forward=RIGHT_IN3, backward=RIGHT_IN4)

    def set_speeds(self, left_speed, right_speed):
        left_speed = max(-1.0, min(1.0, left_speed))
        right_speed = max(-1.0, min(1.0, right_speed))
        self._drive_one(self.left, left_speed)
        self._drive_one(self.right, right_speed)

    @staticmethod
    def _drive_one(motor, speed):
        if speed > 0:
            motor.forward(speed)
        elif speed < 0:
            motor.backward(-speed)
        else:
            motor.stop()

    def stop(self):
        self.left.stop()
        self.right.stop()

    def close(self):
        self.left.close()
        self.right.close()


# ---------------------- Flask dashboard ----------------------

app = Flask(__name__)

PAGE = """
<!DOCTYPE html>
<html>
<head>
<title>curve_follow_centerline -- live tuning</title>
<style>
  body { font-family: sans-serif; margin: 20px; background: #111; color: #eee; }
  .row { display: flex; gap: 30px; flex-wrap: wrap; align-items: flex-start; }
  .controls { min-width: 300px; }
  .slider-group { margin-bottom: 12px; }
  label { display: block; font-size: 13px; margin-bottom: 2px; }
  input[type=range] { width: 240px; }
  .val { font-family: monospace; color: #6cf; }
  #status { margin: 10px 0 16px 0; font-size: 14px; font-family: monospace; }
  img { border: 2px solid #333; max-width: 640px; width: 100%; }
  .btns { margin-bottom: 16px; }
  button { font-size: 16px; padding: 10px 22px; margin-right: 10px; border: none; border-radius: 6px; cursor: pointer; }
  #startBtn { background: #2ecc71; color: #000; }
  #stopBtn { background: #e74c3c; color: #fff; }
  #runState { font-weight: bold; }
  .dpad { display: grid; grid-template-columns: 60px 60px 60px; grid-template-rows: 60px 60px 60px; gap: 6px; margin-top: 20px; user-select: none; }
  .dpad button { width: 60px; height: 60px; font-size: 22px; background: #333; color: #eee; margin: 0; padding: 0; }
  .dpad button:active, .dpad button.held { background: #6cf; color: #000; }
  #manualLabel { margin-top: 10px; font-size: 13px; color: #aaa; max-width: 260px; }
</style>
</head>
<body>
<h2>Centerline Follow -- Live Tuning</h2>
<div class="btns">
  <button id="startBtn">START</button>
  <button id="stopBtn">STOP</button>
  <span id="runState">STOPPED</span>
</div>
<div id="status">Loading...</div>
<div class="row">
  <div class="controls" id="controls">
    <div class="dpad">
      <div></div><button id="btnUp">&uarr;</button><div></div>
      <button id="btnLeft">&larr;</button><button id="btnStop">&#9632;</button><button id="btnRight">&rarr;</button>
      <div></div><button id="btnDown">&darr;</button><div></div>
    </div>
    <div id="manualLabel">Arrow keys or buttons = manual drive (overrides auto, requires pressing START again afterward). Release = stop.</div>
    <div id="sliders"></div>
  </div>
  <div>
    <img src="/stream.mjpg">
  </div>
</div>

<script>
const paramDefs = [
  ["kp", 0, 0.02, 0.0002],
  ["ki", 0, 0.002, 0.00005],
  ["kd", 0, 0.01, 0.0002],
  ["base_speed", 0, 0.8, 0.01],
  ["max_correction", 0, 0.6, 0.01],
  ["heading_weight", 0, 3, 0.05],
  ["lookahead_px", 0, 400, 5],
  ["min_effective_speed", 0, 0.6, 0.01],
  ["manual_speed", 0, 1, 0.01],
];

let currentParams = {};

async function fetchParams() {
  const r = await fetch("/params");
  currentParams = await r.json();
  buildControls();
}

function buildControls() {
  const slidersDiv = document.getElementById("sliders");
  slidersDiv.innerHTML = "";
  for (const [name, min, max, step] of paramDefs) {
    const val = currentParams[name];
    const group = document.createElement("div");
    group.className = "slider-group";
    group.innerHTML = `
      <label>${name}: <span class="val" id="val_${name}">${val}</span></label>
      <input type="range" id="slider_${name}" min="${min}" max="${max}" step="${step}" value="${val}">
    `;
    slidersDiv.appendChild(group);
    const slider = group.querySelector("input");
    slider.addEventListener("input", async (e) => {
      const v = parseFloat(e.target.value);
      document.getElementById(`val_${name}`).textContent = v;
      currentParams[name] = v;
      await fetch("/params", {
        method: "POST",
        headers: {"Content-Type": "application/json"},
        body: JSON.stringify({[name]: v})
      });
    });
  }
}

async function fetchStatus() {
  const r = await fetch("/status");
  const s = await r.json();
  document.getElementById("status").textContent =
    `mode: ${s.mode} | near_error: ${s.near_error !== null ? s.near_error.toFixed(1)+'px' : 'N/A'} | ` +
    `angle: ${s.angle_deg !== null ? s.angle_deg.toFixed(1)+'deg' : 'N/A'} | ` +
    `L: ${s.left_speed !== null ? s.left_speed.toFixed(2) : 'N/A'} | ` +
    `R: ${s.right_speed !== null ? s.right_speed.toFixed(2) : 'N/A'} | ` +
    `lost_streak: ${s.lost_streak}`;
  const runState = document.getElementById("runState");
  runState.textContent = s.mode.toUpperCase();
  runState.style.color = s.mode === "auto" ? "#2ecc71" : (s.mode === "manual" ? "#6cf" : "#e74c3c");
}

document.getElementById("startBtn").addEventListener("click", async () => {
  await fetch("/start", {method: "POST"});
});
document.getElementById("stopBtn").addEventListener("click", async () => {
  await fetch("/stop", {method: "POST"});
});

// ---- Manual drive (arrow keys + on-screen dpad) ----
const heldKeys = new Set();
let manualInterval = null;

function computeManualSpeeds() {
  const speed = currentParams.manual_speed !== undefined ? currentParams.manual_speed : 0.4;
  const forward = (heldKeys.has("ArrowUp") ? 1 : 0) - (heldKeys.has("ArrowDown") ? 1 : 0);
  const turn = (heldKeys.has("ArrowRight") ? 1 : 0) - (heldKeys.has("ArrowLeft") ? 1 : 0);
  let left = forward * speed + turn * speed;
  let right = forward * speed - turn * speed;
  left = Math.max(-1, Math.min(1, left));
  right = Math.max(-1, Math.min(1, right));
  return {left, right};
}

async function sendManual() {
  const {left, right} = computeManualSpeeds();
  await fetch("/manual", {
    method: "POST",
    headers: {"Content-Type": "application/json"},
    body: JSON.stringify({left, right})
  });
}

function startManualLoop() {
  if (manualInterval) return;
  sendManual();
  manualInterval = setInterval(sendManual, 150);
}

function stopManualLoopIfIdle() {
  if (heldKeys.size === 0) {
    if (manualInterval) { clearInterval(manualInterval); manualInterval = null; }
    fetch("/manual", {
      method: "POST",
      headers: {"Content-Type": "application/json"},
      body: JSON.stringify({left: 0, right: 0})
    });
  }
}

function pressKey(key) {
  heldKeys.add(key);
  highlightButtons();
  startManualLoop();
}
function releaseKey(key) {
  heldKeys.delete(key);
  highlightButtons();
  stopManualLoopIfIdle();
}

function highlightButtons() {
  const map = {ArrowUp: "btnUp", ArrowDown: "btnDown", ArrowLeft: "btnLeft", ArrowRight: "btnRight"};
  for (const [key, id] of Object.entries(map)) {
    document.getElementById(id).classList.toggle("held", heldKeys.has(key));
  }
}

document.addEventListener("keydown", (e) => {
  if (["ArrowUp","ArrowDown","ArrowLeft","ArrowRight"].includes(e.key)) {
    e.preventDefault();
    if (!e.repeat) pressKey(e.key);
  }
});
document.addEventListener("keyup", (e) => {
  if (["ArrowUp","ArrowDown","ArrowLeft","ArrowRight"].includes(e.key)) {
    e.preventDefault();
    releaseKey(e.key);
  }
});

function wireButton(id, key) {
  const el = document.getElementById(id);
  const start = (e) => { e.preventDefault(); pressKey(key); };
  const end = (e) => { e.preventDefault(); releaseKey(key); };
  el.addEventListener("mousedown", start);
  el.addEventListener("touchstart", start);
  el.addEventListener("mouseup", end);
  el.addEventListener("mouseleave", end);
  el.addEventListener("touchend", end);
}
wireButton("btnUp", "ArrowUp");
wireButton("btnDown", "ArrowDown");
wireButton("btnLeft", "ArrowLeft");
wireButton("btnRight", "ArrowRight");
document.getElementById("btnStop").addEventListener("click", () => {
  heldKeys.clear();
  highlightButtons();
  stopManualLoopIfIdle();
});

fetchParams();
setInterval(fetchStatus, 300);
</script>
</body>
</html>
"""


def generate_mjpeg():
    global latest_jpeg
    while True:
        with stream_lock:
            frame_bytes = latest_jpeg
        if frame_bytes is not None:
            yield (b'--frame\r\n'
                   b'Content-Type: image/jpeg\r\n\r\n' + frame_bytes + b'\r\n')
        time.sleep(0.03)


@app.route('/stream.mjpg')
def stream():
    return Response(generate_mjpeg(), mimetype='multipart/x-mixed-replace; boundary=frame')


@app.route('/')
def index():
    return Response(PAGE, mimetype="text/html")


@app.route('/params', methods=["GET", "POST"])
def params_route():
    if request.method == "POST":
        data = request.get_json()
        with params_lock:
            for k, v in data.items():
                if k in params:
                    params[k] = float(v)
        return jsonify({"ok": True})
    with params_lock:
        return jsonify(params)


@app.route('/status')
def status_route():
    with status_lock:
        return jsonify(status)


@app.route('/start', methods=["POST"])
def start_route():
    global control_enabled
    with control_lock:
        control_enabled = True
    print(">>> START pressed -- motors enabled")
    return jsonify({"ok": True})


@app.route('/stop', methods=["POST"])
def stop_route():
    global control_enabled
    with control_lock:
        control_enabled = False
    print(">>> STOP pressed -- motors disabled")
    return jsonify({"ok": True})


@app.route('/manual', methods=["POST"])
def manual_route():
    global manual_left, manual_right, manual_until, control_enabled
    data = request.get_json()
    left = float(data.get("left", 0.0))
    right = float(data.get("right", 0.0))
    with manual_lock:
        manual_left = left
        manual_right = right
        manual_until = time.time() + MANUAL_TIMEOUT
    # Manual input always takes over from auto -- avoid the two fighting
    # for the motors. User must press START again to resume auto-follow.
    with control_lock:
        control_enabled = False
    return jsonify({"ok": True})


def run_flask():
    app.run(host="0.0.0.0", port=8000, debug=False, use_reloader=False, threaded=True)


# ---------------------- Control loop ----------------------

def main():
    global latest_jpeg

    picam2 = Picamera2()
    config = picam2.create_video_configuration(
        main={"size": (FRAME_W, FRAME_H), "format": "RGB888"},
        transform=Transform(hflip=1, vflip=1)
    )
    picam2.configure(config)
    picam2.start()
    time.sleep(1)

    drive = None if DRY_RUN else DiffDrive()

    if ENABLE_STREAM:
        flask_thread = threading.Thread(target=run_flask, daemon=True)
        flask_thread.start()
        print("Streaming enabled: http://<pi-ip>:8000/stream.mjpg")

    print(f"Starting curve_follow_centerline. DRY_RUN={DRY_RUN} ENABLE_STREAM={ENABLE_STREAM}. Ctrl+C to stop.\n")

    integral = 0.0
    prev_error = 0.0
    prev_time = time.time()
    lost_streak = 0
    frame_count = 0
    t_start = time.time()

    try:
        while True:
            frame = picam2.capture_array()
            frame_bgr = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)

            near_error, slope, angle_deg, num_pts, debug = detect_centerline(
                frame_bgr, draw_debug=ENABLE_STREAM)
            frame_count += 1

            now = time.time()
            dt = now - prev_time
            prev_time = now

            with manual_lock:
                manual_active = now < manual_until
                ml, mr = manual_left, manual_right

            if manual_active:
                lost_streak = 0
                integral = 0.0  # avoid stale windup when auto resumes later

                if drive:
                    drive.set_speeds(ml, mr)

                with status_lock:
                    status["near_error"] = float(near_error) if near_error is not None else None
                    status["angle_deg"] = float(angle_deg) if angle_deg is not None else None
                    status["left_speed"] = float(ml)
                    status["right_speed"] = float(mr)
                    status["lost_streak"] = 0
                    status["mode"] = "manual"

                if frame_count % 5 == 0:
                    print(f"MANUAL L={ml:+.2f} R={mr:+.2f}")

                if debug is not None:
                    cv2.putText(debug, f"MANUAL L={ml:+.2f} R={mr:+.2f}",
                                (10, 86), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 200, 0), 2)

            elif near_error is not None:
                lost_streak = 0

                with params_lock:
                    p = dict(params)

                combined_error = near_error + p["heading_weight"] * slope * p["lookahead_px"]

                integral += combined_error * dt
                integral = max(-INTEGRAL_CLAMP, min(INTEGRAL_CLAMP, integral))
                derivative = (combined_error - prev_error) / dt if dt > 0 else 0.0
                prev_error = combined_error

                correction = p["kp"] * combined_error + p["ki"] * integral + p["kd"] * derivative
                correction = max(-p["max_correction"], min(p["max_correction"], correction))

                left_speed = p["base_speed"] + correction
                right_speed = p["base_speed"] - correction
                left_speed, right_speed = apply_min_effective_speed_pair(
                    left_speed, right_speed, p["min_effective_speed"])

                with control_lock:
                    enabled = control_enabled

                if drive and enabled:
                    drive.set_speeds(left_speed, right_speed)
                elif drive and not enabled:
                    drive.stop()

                with status_lock:
                    status["near_error"] = float(near_error)
                    status["angle_deg"] = float(angle_deg)
                    status["left_speed"] = float(left_speed)
                    status["right_speed"] = float(right_speed)
                    status["lost_streak"] = 0
                    status["mode"] = "auto" if enabled else "stopped"

                if frame_count % 5 == 0:
                    print(f"near_err={near_error:+6.1f} angle={angle_deg:+5.1f}deg "
                          f"combined={combined_error:+7.2f} correction={correction:+.3f} "
                          f"L={left_speed:+.2f} R={right_speed:+.2f} enabled={enabled}")

                if debug is not None:
                    cv2.putText(debug, f"L={left_speed:+.2f} R={right_speed:+.2f}",
                                (10, 86), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 200, 0), 2)
                    run_txt = "RUNNING" if enabled else "STOPPED"
                    run_color = (0, 200, 0) if enabled else (0, 0, 255)
                    cv2.putText(debug, run_txt, (10, 114), cv2.FONT_HERSHEY_SIMPLEX, 0.7, run_color, 2)
            else:
                lost_streak += 1
                with status_lock:
                    status["lost_streak"] = lost_streak
                    status["mode"] = "stopped"
                if lost_streak >= MAX_LOST_FRAMES:
                    print(f"LINE LOST for {lost_streak} frames -- SAFETY STOP")
                    if drive:
                        drive.stop()
                    integral = 0.0
                else:
                    print(f"Line lost ({lost_streak}/{MAX_LOST_FRAMES}) -- holding last command")

            if ENABLE_STREAM and debug is not None:
                ok, jpeg = cv2.imencode('.jpg', debug, [cv2.IMWRITE_JPEG_QUALITY, JPEG_QUALITY])
                if ok:
                    with stream_lock:
                        latest_jpeg = jpeg.tobytes()

    except KeyboardInterrupt:
        elapsed = time.time() - t_start
        print(f"\nStopped by user. {frame_count} frames in {elapsed:.1f}s "
              f"({frame_count/elapsed:.1f} fps avg).")
    finally:
        if drive:
            drive.stop()
            drive.close()
        picam2.stop()


if __name__ == "__main__":
    main()
