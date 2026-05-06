"""
Headless CSI-camera ball detection + platform rotation.

Note: Ultralytics imports OpenCV (`cv2`). On some Jetson/conda setups, importing cv2 can fail with:
  libgdk_pixbuf-2.0.so.0: undefined symbol: g_task_set_static_name
because `libgdk_pixbuf` ends up linking against the system `libgio` instead of the conda one.
We proactively preload conda's libgio with RTLD_GLOBAL before importing ultralytics/cv2.
"""

import os
import ctypes

_conda_prefix = os.environ.get("CONDA_PREFIX")
if _conda_prefix:
    _libgio = os.path.join(_conda_prefix, "lib", "libgio-2.0.so.0")
    if os.path.exists(_libgio):
        try:
            ctypes.CDLL(_libgio, mode=ctypes.RTLD_GLOBAL)
        except OSError:
            # If this fails, we'll see the original import error below.
            pass

from ultralytics import YOLO
import cv2
import torch
import threading
from http import server
from socketserver import ThreadingMixIn
import time
import numpy as np
import subprocess
import signal

from move_platform import ROBOT_ID, PORT, LeKiwi, LeKiwiConfig, configure_wheels, rotate_platform

# Load your custom model
model = YOLO("./balls.pt")
YOLO_DEVICE = 0 if torch.cuda.is_available() else "cpu"
YOLO_HALF = torch.cuda.is_available()

CSI_SENSOR_ID = 0
CSI_FLIP_METHOD = 0
CSI_WIDTH = 320
CSI_HEIGHT = 240
CSI_FPS = 10
TCP_HOST = "127.0.0.1"
TCP_PORT = 5000
SYSTEM_GST_LAUNCH = "/usr/bin/gst-launch-1.0"
SYSTEM_GST_INSPECT = "/usr/bin/gst-inspect-1.0"
SYSTEM_GST_PLUGIN_SCANNER = "/usr/lib/aarch64-linux-gnu/gstreamer1.0/gstreamer-1.0/gst-plugin-scanner"
SYSTEM_GST_PLUGIN_PATH = "/usr/lib/aarch64-linux-gnu/gstreamer-1.0"
YOLO_IMGSZ = 224
DETECT_EVERY_N_FRAMES = 5


def _build_system_gst_env() -> dict:
    env = os.environ.copy()
    # Let system gst-launch use system GLib/GStreamer, not conda's injected runtime.
    env.pop("LD_PRELOAD", None)
    env.pop("CONDA_PREFIX", None)
    env.pop("CONDA_DEFAULT_ENV", None)
    env.pop("GST_PLUGIN_PATH", None)
    env.pop("GST_PLUGIN_SYSTEM_PATH", None)
    env["LD_LIBRARY_PATH"] = "/usr/lib/aarch64-linux-gnu:/lib/aarch64-linux-gnu:/usr/lib:/lib"
    if os.path.exists(SYSTEM_GST_PLUGIN_SCANNER):
        env["GST_PLUGIN_SCANNER"] = SYSTEM_GST_PLUGIN_SCANNER
    if os.path.exists(SYSTEM_GST_PLUGIN_PATH):
        env["GST_PLUGIN_SYSTEM_PATH"] = SYSTEM_GST_PLUGIN_PATH
    return env


def _probe_csi_camera(env: dict) -> str | None:
    inspect = subprocess.run(
        [SYSTEM_GST_INSPECT, "nvarguscamerasrc"],
        env=env,
        stdout=subprocess.DEVNULL,
        stderr=subprocess.PIPE,
        text=True,
    )
    if inspect.returncode != 0:
        message = inspect.stderr.strip() or "System GStreamer could not find nvarguscamerasrc."
        return f"CSI camera support is unavailable: {message}"

    probe = subprocess.run(
        [
            SYSTEM_GST_LAUNCH,
            "-q",
            "nvarguscamerasrc",
            f"sensor-id={CSI_SENSOR_ID}",
            "num-buffers=1",
            "!",
            "fakesink",
        ],
        env=env,
        stdout=subprocess.DEVNULL,
        stderr=subprocess.PIPE,
        text=True,
    )
    if probe.returncode == 0:
        return None

    message = probe.stderr.strip() or "Unknown Argus camera failure."
    if "No cameras available" in message or "Sensor could not be opened" in message:
        return (
            "Argus cannot detect/open the CSI camera. "
            "Check the ribbon cable orientation and seating, the correct CSI port, "
            "and whether this camera module is supported by the current Jetson image/device-tree."
        )
    return f"CSI camera probe failed: {message}"


def _start_csi_to_tcp_mpegts_stream() -> subprocess.Popen:
    """
    Use *system* GStreamer (with nvarguscamerasrc) to publish the CSI camera as H264 in MPEG-TS over TCP.
    OpenCV in this conda env does not have GStreamer enabled, but it *does* have FFmpeg, so we read
    the stream using cv2.CAP_FFMPEG from tcp://.
    """
    gst_cmd = [
        SYSTEM_GST_LAUNCH,
        "-q",
        f"nvarguscamerasrc",
        f"sensor-id={CSI_SENSOR_ID}",
        "!",
        f"video/x-raw(memory:NVMM),width={CSI_WIDTH},height={CSI_HEIGHT},format=NV12,framerate={CSI_FPS}/1",
        "!",
        "nvvidconv",
        f"flip-method={CSI_FLIP_METHOD}",
        "!",
        "video/x-raw,format=I420",
        "!",
        "x264enc",
        "tune=zerolatency",
        "speed-preset=ultrafast",
        "bitrate=4000",
        "bframes=0",
        "key-int-max=15",
        "byte-stream=true",
        "!",
        "h264parse",
        "config-interval=1",
        "!",
        "mpegtsmux",
        "!",
        "tcpserversink",
        f"host={TCP_HOST}",
        f"port={TCP_PORT}",
        "sync=false",
    ]
    # Keep stderr for debugging if the pipeline fails to start.
    gst_log_path = "/tmp/detect_ball_gst.log"
    gst_log = open(gst_log_path, "w")  # noqa: SIM115
    gst_env = _build_system_gst_env()
    csi_error = _probe_csi_camera(gst_env)
    if csi_error is not None:
        gst_log.close()
        raise RuntimeError(csi_error)
    proc = subprocess.Popen(gst_cmd, stdout=subprocess.DEVNULL, stderr=gst_log, env=gst_env)
    print(f"[camera] Started gst-launch pipeline (logs: {gst_log_path})")
    return proc


_gst_proc = _start_csi_to_tcp_mpegts_stream()
time.sleep(0.5)  # allow pipeline to start

# Using tcp:// avoids OpenCV FFmpeg UDP capture issues on some builds.
# Retry for a short while because gst-launch may take a moment to bind the port.
cap = None
deadline = time.time() + 5.0
while time.time() < deadline:
    if _gst_proc.poll() is not None:
        raise RuntimeError(
            "gst-launch exited early; see /tmp/detect_ball_gst.log for details."
        )
    cap = cv2.VideoCapture(f"tcp://{TCP_HOST}:{TCP_PORT}", cv2.CAP_FFMPEG)
    if cap.isOpened():
        break
    cap.release()
    time.sleep(0.2)

if cap is None or not cap.isOpened():
    if _gst_proc.poll() is None:
        _gst_proc.terminate()
    raise RuntimeError("Failed to open TCP MPEG-TS stream in OpenCV. See /tmp/detect_ball_gst.log.")

CONF_THRESHOLD = 0.2
GREEN_CLASS_INDEX = 2  # index in `colors` that corresponds to "green"
CAMERA_H_FOV_DEG = 60  # adjust to your camera's horizontal field of view

STREAM_HOST = "0.0.0.0"
STREAM_PORT = 8000
JPEG_QUALITY = 45
STREAM_EVERY_N_FRAMES = 2

robot = LeKiwi(LeKiwiConfig(id=ROBOT_ID, port=PORT))
bus = robot.bus
bus.connect()
configure_wheels(bus)

_latest_jpeg: bytes | None = None
_jpeg_lock = threading.Lock()


class _ThreadingHTTPServer(ThreadingMixIn, server.HTTPServer):
    daemon_threads = True


class _StreamHandler(server.BaseHTTPRequestHandler):
    def do_GET(self):  # noqa: N802
        if self.path in ("/", "/index.html"):
            html = (
                "<html><body>"
                "<h3>detect_ball stream</h3>"
                "<img src='/mjpeg' />"
                "</body></html>"
            ).encode("utf-8")
            self.send_response(200)
            self.send_header("Content-Type", "text/html; charset=utf-8")
            self.send_header("Content-Length", str(len(html)))
            self.end_headers()
            self.wfile.write(html)
            return

        if self.path == "/jpeg":
            with _jpeg_lock:
                frame = _latest_jpeg
            if frame is None:
                self.send_response(503)
                self.end_headers()
                return
            self.send_response(200)
            self.send_header("Content-Type", "image/jpeg")
            self.send_header("Content-Length", str(len(frame)))
            self.end_headers()
            self.wfile.write(frame)
            return

        if self.path == "/mjpeg":
            self.send_response(200)
            self.send_header("Cache-Control", "no-cache, private")
            self.send_header("Pragma", "no-cache")
            self.send_header("Content-Type", "multipart/x-mixed-replace; boundary=frame")
            self.end_headers()
            try:
                while True:
                    with _jpeg_lock:
                        frame = _latest_jpeg
                    if frame is None:
                        time.sleep(0.05)
                        continue
                    self.wfile.write(b"--frame\r\n")
                    self.wfile.write(b"Content-Type: image/jpeg\r\n")
                    self.wfile.write(f"Content-Length: {len(frame)}\r\n\r\n".encode("utf-8"))
                    self.wfile.write(frame)
                    self.wfile.write(b"\r\n")
                    time.sleep(0.05)
            except BrokenPipeError:
                return

        self.send_response(404)
        self.end_headers()

    def log_message(self, format, *args):  # noqa: A002
        # Keep stdout clean (YOLO already logs)
        return


def _start_stream_server() -> None:
    httpd = _ThreadingHTTPServer((STREAM_HOST, STREAM_PORT), _StreamHandler)
    t = threading.Thread(target=httpd.serve_forever, daemon=True)
    t.start()
    print(f"[stream] MJPEG: http://localhost:{STREAM_PORT}/  (or http://<robot_ip>:{STREAM_PORT}/)")


_start_stream_server()


def calculate_depth(radius):
    if radius > 0:
        # Als r = 11, dan is de afstand 156 cm
        # Depth moet kleiner worden als de radius groter is (omgekeerd evenredig)
        # Bijvoorbeeld: depth = k / radius, waarbij k een kalibratieconstante is
        # Gegeven: als r = 11, dan is de afstand 156 cm -> k = 11 * 156 = 1716
        depth = 1716 / radius
        return round(depth, 1)


def compute_rotation_angle(frame_width, target_x):
    """
    Compute platform rotation angle so that the camera center points toward target_x.
    Positive angle => rotate right, negative => rotate left (convention can be adapted).
    """
    center_x = frame_width / 2.0
    pixel_offset = target_x - center_x  # >0 if ball is to the right of center
    norm_offset = pixel_offset / (frame_width / 2.0)  # -1 .. 1
    angle = norm_offset * (CAMERA_H_FOV_DEG / 2.0)
    return angle

try:
    frame_idx = 0
    best_conf = -1.0
    best_center = None
    best_box = None
    diameter = None

    while True:
        frame_idx += 1
        ret, frame = cap.read()
        if not ret:
            break

        # Run detection only every N frames and reuse the latest result in between.
        if frame_idx % DETECT_EVERY_N_FRAMES == 0:
            results = model(
                frame,
                verbose=False,
                imgsz=YOLO_IMGSZ,
                device=YOLO_DEVICE,
                half=YOLO_HALF,
                classes=[GREEN_CLASS_INDEX],
                conf=CONF_THRESHOLD,
            )

            best_conf = -1.0
            best_center = None
            best_box = None
            diameter = None
            for r in results:
                for box in r.boxes:
                    conf = float(box.conf[0])

                    x1, y1, x2, y2 = map(int, box.xyxy[0])
                    cls = int(box.cls[0])

                    if cls == GREEN_CLASS_INDEX and conf > best_conf:
                        cx = int((x1 + x2) / 2)
                        cy = int((y1 + y2) / 2)
                        best_conf = conf
                        best_center = (cx, cy)
                        best_box = (x1, y1, x2, y2)
                        diameter = x2 - x1

        # If we found a green ball this frame, draw it and save the center
        if best_center is not None and best_box is not None:
            x1, y1, x2, y2 = best_box
            cx, cy = best_center

            # Draw bounding box + center + info (this is what the stream shows)
            cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
            cv2.circle(frame, (cx, cy), 5, (0, 255, 0), -1)
            if diameter is not None:
                cv2.putText(
                    frame,
                    f"green: {best_conf:.2f} | depth: {calculate_depth(diameter/2)} cm",
                    (x1, max(0, y1 - 10)),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.6,
                    (0, 255, 0),
                    2,
                )

            # Compute required rotation and command platform
            frame_h, frame_w = frame.shape[:2]
            angle = compute_rotation_angle(frame_w, cx)
            rotate_platform(bus, angle_deg=angle)

        # Update stream
        if frame_idx % STREAM_EVERY_N_FRAMES == 0:
            ok, buf = cv2.imencode(".jpg", frame, [int(cv2.IMWRITE_JPEG_QUALITY), JPEG_QUALITY])
            if ok:
                with _jpeg_lock:
                    _latest_jpeg = buf.tobytes()

except KeyboardInterrupt:
    print("Stopping detect_ball...")
finally:
    cap.release()
    if bus.is_connected:
        bus.disconnect()
    if _gst_proc.poll() is None:
        _gst_proc.send_signal(signal.SIGINT)
        try:
            _gst_proc.wait(timeout=2.0)
        except subprocess.TimeoutExpired:
            _gst_proc.kill()