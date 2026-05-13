"""
Camera module for CSI camera streaming and ball detection via YOLO.
Handles GStreamer pipeline, OpenCV video capture, and YOLO inference.
"""

import os
import time
import subprocess
import ctypes

import cv2
import torch
from ultralytics import YOLO

# Preload libgio for conda/ultralytics compatibility
_conda_prefix = os.environ.get("CONDA_PREFIX")
if _conda_prefix:
    _libgio = os.path.join(_conda_prefix, "lib", "libgio-2.0.so.0")
    if os.path.exists(_libgio):
        try:
            ctypes.CDLL(_libgio, mode=ctypes.RTLD_GLOBAL)
        except OSError:
            pass

# Global state
_cap = None
_model = None
_gst_proc = None

# Detection constants
YOLO_MODEL_PATH = "./balls.pt"
YOLO_DEVICE = 0 if torch.cuda.is_available() else "cpu"
YOLO_HALF = torch.cuda.is_available()
YOLO_IMGSZ = 224
CONF_THRESHOLD = 0.2
GREEN_CLASS_INDEX = 2
CAMERA_H_FOV_DEG = 60

# CSI camera config
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
SYSTEM_GST_PLUGIN_PATH = "/usr/lib/aarch64-linux-gnu/gstreamer1.0"


def _build_system_gst_env() -> dict:
    """Build environment for system GStreamer."""
    env = os.environ.copy()
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
    """Probe if CSI camera is available."""
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
    """Start gstreamer pipeline for CSI camera over TCP."""
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
    gst_log_path = "/tmp/detect_ball_gst.log"
    gst_log = open(gst_log_path, "w")
    gst_env = _build_system_gst_env()
    csi_error = _probe_csi_camera(gst_env)
    if csi_error is not None:
        gst_log.close()
        raise RuntimeError(csi_error)
    proc = subprocess.Popen(gst_cmd, stdout=subprocess.DEVNULL, stderr=gst_log, env=gst_env)
    print(f"[camera] Started gst-launch pipeline (logs: {gst_log_path})")
    return proc


def initialize_camera():
    """Initialize camera and YOLO model."""
    global _cap, _model, _gst_proc
    
    if _cap is not None and _model is not None:
        return
    
    try:
        _gst_proc = _start_csi_to_tcp_mpegts_stream()
        time.sleep(0.5)
        
        _cap = None
        deadline = time.time() + 5.0
        while time.time() < deadline:
            if _gst_proc.poll() is not None:
                raise RuntimeError("gst-launch exited early")
            _cap = cv2.VideoCapture(f"tcp://{TCP_HOST}:{TCP_PORT}", cv2.CAP_FFMPEG)
            if _cap.isOpened():
                break
            _cap.release()
            time.sleep(0.2)
        
        if _cap is None or not _cap.isOpened():
            raise RuntimeError("Failed to open TCP MPEG-TS stream")
        
        print("[camera] Camera stream opened successfully")
        
        _model = YOLO(YOLO_MODEL_PATH)
        print(f"[model] Loaded YOLO model from {YOLO_MODEL_PATH}")
        
    except Exception as e:
        print(f"Error initializing camera: {e}")
        raise


def calculate_depth(radius: float) -> float:
    """Calculate depth from ball radius in pixels."""
    if radius > 0:
        depth = 1716 / radius
        return round(depth, 1)
    return None


def detect_ball(frame) -> tuple[float, float, float] | None:
    """
    Detect ball in frame and return (x, y, z) in camera coordinates.
    x: horizontal offset from center (cm)
    y: forward/depth (cm, negative in this codebase)
    z: height (cm)
    """
    if _model is None or frame is None:
        return None
    
    try:
        results = _model(
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
                    diameter = x2 - x1
        
        if best_center is None:
            return None
        
        cx, cy = best_center
        frame_h, frame_w = frame.shape[:2]
        
        # Convert pixel coordinates to camera coordinates (cm)
        pixel_to_cm = 0.1  # Rough calibration; adjust based on your camera
        x_cm = (cx - frame_w / 2.0) * pixel_to_cm
        z_cm = 0.0  # Set to 0 for now; could use cy for vertical offset
        
        # Calculate depth from radius
        radius = diameter / 2.0
        depth_cm = calculate_depth(radius)
        y_cm = -depth_cm if depth_cm else None
        
        if y_cm is None:
            return None
        
        return (x_cm, y_cm, z_cm)
        
    except Exception as e:
        print(f"Error in detection: {e}")
        return None


def get_video_capture():
    """Get the video capture object."""
    global _cap
    return _cap
