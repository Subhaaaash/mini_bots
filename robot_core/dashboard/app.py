"""Generic Flask + MJPEG dashboard.

Unlike the original script, this dashboard knows nothing about line
following specifically: its tuning sliders are built from whatever
`behavior.param_specs()` / `get_params()` / `set_params()` report, so any
new Behavior automatically gets a working dashboard for free. Start/stop and
manual d-pad drive talk to a SafetyGovernor, also independent of which
Behavior is running.

The main control loop calls update_status()/update_frame() every frame; the
Flask routes below just read that latest snapshot -- same
producer/consumer-via-locks pattern as the original script.
"""

import threading
import time

from flask import Flask, Response, jsonify, request

from robot_core.behaviors.base import Behavior
from robot_core.safety.governor import SafetyGovernor

PAGE = """
<!DOCTYPE html>
<html>
<head>
<title>{title} -- live tuning</title>
<style>
  body { font-family: sans-serif; margin: 20px; background: #111; color: #eee; }
  .row { display: flex; gap: 30px; flex-wrap: wrap; align-items: flex-start; }
  .controls { min-width: 300px; }
  .slider-group { margin-bottom: 12px; }
  label { display: block; font-size: 13px; margin-bottom: 2px; }
  input[type=range] { width: 240px; }
  .val { font-family: monospace; color: #6cf; }
  #status { margin: 10px 0 16px 0; font-size: 14px; font-family: monospace; white-space: pre-wrap; }
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
<h2>{title} -- Live Tuning</h2>
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
    <div class="slider-group">
      <label>manual_speed: <span class="val" id="val_manual_speed">0.4</span></label>
      <input type="range" id="slider_manual_speed" min="0" max="1" step="0.01" value="0.4">
    </div>
    <div id="sliders"></div>
  </div>
  <div>
    <img src="/stream.mjpg">
  </div>
</div>

<script>
let paramDefs = [];
let currentParams = {};
let manualSpeed = 0.4;

async function fetchParamSpecs() {
  const r = await fetch("/param_specs");
  const specs = await r.json();
  paramDefs = Object.entries(specs).map(([name, [min, max, step]]) => [name, min, max, step]);
}

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

document.getElementById("slider_manual_speed").addEventListener("input", (e) => {
  manualSpeed = parseFloat(e.target.value);
  document.getElementById("val_manual_speed").textContent = manualSpeed;
});

async function fetchStatus() {
  const r = await fetch("/status");
  const s = await r.json();
  const extras = Object.entries(s)
    .filter(([k]) => !["mode", "lost_streak"].includes(k))
    .map(([k, v]) => `${k}: ${v !== null && v !== undefined ? (typeof v === "number" ? v.toFixed(2) : v) : "N/A"}`)
    .join(" | ");
  document.getElementById("status").textContent =
    `mode: ${s.mode} | lost_streak: ${s.lost_streak}` + (extras ? ` | ${extras}` : "");
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
  const forward = (heldKeys.has("ArrowUp") ? 1 : 0) - (heldKeys.has("ArrowDown") ? 1 : 0);
  const turn = (heldKeys.has("ArrowRight") ? 1 : 0) - (heldKeys.has("ArrowLeft") ? 1 : 0);
  let left = forward * manualSpeed + turn * manualSpeed;
  let right = forward * manualSpeed - turn * manualSpeed;
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

(async () => {
  await fetchParamSpecs();
  await fetchParams();
})();
setInterval(fetchStatus, 300);
</script>
</body>
</html>
"""


class DashboardApp:
    """Owns the Flask app + MJPEG stream. The control loop pushes frames and
    status via update_frame()/update_status(); Flask request handlers read
    those and call into `behavior` / `governor`.
    """

    def __init__(self, behavior: Behavior, governor: SafetyGovernor, title: str = "robot_core"):
        self.behavior = behavior
        self.governor = governor
        self.title = title

        self._stream_lock = threading.Lock()
        self._latest_jpeg: bytes | None = None
        self._status_lock = threading.Lock()
        self._status: dict = {"mode": "stopped", "lost_streak": 0}

        self.app = Flask(__name__)
        self._register_routes()

    def update_frame(self, jpeg_bytes: bytes) -> None:
        with self._stream_lock:
            self._latest_jpeg = jpeg_bytes

    def update_status(self, status: dict) -> None:
        with self._status_lock:
            self._status = dict(status)

    def run(self, host: str = "0.0.0.0", port: int = 8000) -> None:
        self.app.run(host=host, port=port, debug=False, use_reloader=False, threaded=True)

    def run_in_thread(self, host: str = "0.0.0.0", port: int = 8000) -> threading.Thread:
        t = threading.Thread(target=self.run, kwargs={"host": host, "port": port}, daemon=True)
        t.start()
        return t

    def _generate_mjpeg(self):
        while True:
            with self._stream_lock:
                frame_bytes = self._latest_jpeg
            if frame_bytes is not None:
                yield (b"--frame\r\nContent-Type: image/jpeg\r\n\r\n" + frame_bytes + b"\r\n")
            time.sleep(0.03)

    def _register_routes(self):
        app = self.app

        @app.route("/")
        def index():
            return Response(PAGE.replace("{title}", self.title), mimetype="text/html")

        @app.route("/stream.mjpg")
        def stream():
            return Response(self._generate_mjpeg(), mimetype="multipart/x-mixed-replace; boundary=frame")

        @app.route("/param_specs")
        def param_specs():
            return jsonify(self.behavior.param_specs())

        @app.route("/params", methods=["GET", "POST"])
        def params_route():
            if request.method == "POST":
                data = request.get_json()
                self.behavior.set_params(data)
                return jsonify({"ok": True})
            return jsonify(self.behavior.get_params())

        @app.route("/status")
        def status_route():
            with self._status_lock:
                return jsonify(self._status)

        @app.route("/start", methods=["POST"])
        def start_route():
            self.governor.start()
            return jsonify({"ok": True})

        @app.route("/stop", methods=["POST"])
        def stop_route():
            self.governor.stop()
            return jsonify({"ok": True})

        @app.route("/manual", methods=["POST"])
        def manual_route():
            data = request.get_json()
            left = float(data.get("left", 0.0))
            right = float(data.get("right", 0.0))
            self.governor.set_manual(left, right, time.time())
            return jsonify({"ok": True})
