"""
Camera module: CSI via system GStreamer (nvarguscamerasrc) → MPEG-TS/TCP → OpenCV FFmpeg → BGR + YOLO.

Conda OpenCV often lacks a working CAP_GSTREAMER appsink path; we use /usr/bin/gst-launch-1.0
with Jetson plugins and read the stream with cv2.CAP_FFMPEG (same approach as before).
"""

import json
import os
import subprocess
import time
import ctypes
from pathlib import Path

import cv2
import numpy as np
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

_cap = None
_model = None
_gst_proc: subprocess.Popen | None = None
_loaded_model_path: str | None = None
_color_matrix: np.ndarray | None = None
_color_matrix_loaded: bool = False

CALIBRATION_FILE = "color_calibration.json"
DEFAULT_YOLO_MODEL_PATH = "./balls_ourdata_augmented.pt"

YOLO_DEVICE = 0 if torch.cuda.is_available() else "cpu"
YOLO_HALF = torch.cuda.is_available()
YOLO_IMGSZ = 224
CONF_THRESHOLD = 0.2
GREEN_CLASS_INDEX = 0
CAMERA_H_FOV_DEG = 60

# CSI — match lerobot/examples/phone_to_so100/arm_camera.py (640x360 @ 30)
CSI_SENSOR_ID = 0
CSI_FLIP_METHOD = 0
CSI_WIDTH = 640
CSI_HEIGHT = 360
CSI_FPS = 30

# Argus → nvvidconv → BGRx → BGR (arm_camera.py). TCP path uses I420+x264; we force the same WxH and 1:1 PAR.

TCP_STREAM_BIND_HOST = "0.0.0.0"
TCP_STREAM_CLIENT_HOST = "127.0.0.1"
TCP_PORT = 5000
SYSTEM_GST_LAUNCH = "/usr/bin/gst-launch-1.0"
SYSTEM_GST_INSPECT = "/usr/bin/gst-inspect-1.0"
SYSTEM_GST_PLUGIN_SCANNER = (
    "/usr/lib/aarch64-linux-gnu/gstreamer1.0/gstreamer-1.0/gst-plugin-scanner"
)
SYSTEM_GSTREAMER_PLUGINS = "/usr/lib/aarch64-linux-gnu/gstreamer-1.0"
GST_LOG_PATH = "/tmp/detect_ball_gst.log"


def _build_system_gst_env() -> dict:
    env = os.environ.copy()
    env.pop("LD_PRELOAD", None)
    env.pop("CONDA_PREFIX", None)
    env.pop("CONDA_DEFAULT_ENV", None)
    env.pop("GST_PLUGIN_PATH", None)
    env.pop("GST_PLUGIN_SYSTEM_PATH", None)
    env.pop("GST_PLUGIN_PATH_1_0", None)
    env["LD_LIBRARY_PATH"] = "/usr/lib/aarch64-linux-gnu:/lib/aarch64-linux-gnu:/usr/lib:/lib"
    if os.path.exists(SYSTEM_GST_PLUGIN_SCANNER):
        env["GST_PLUGIN_SCANNER"] = SYSTEM_GST_PLUGIN_SCANNER
    if os.path.isdir(SYSTEM_GSTREAMER_PLUGINS):
        env["GST_PLUGIN_SYSTEM_PATH"] = SYSTEM_GSTREAMER_PLUGINS
        env["GST_PLUGIN_PATH"] = SYSTEM_GSTREAMER_PLUGINS
    return env


def _resolve_model_path(model_path: str | os.PathLike | None) -> str:
    base = model_path if model_path is not None else DEFAULT_YOLO_MODEL_PATH
    return str(Path(base).expanduser().resolve())


def _calibration_json_path() -> Path:
    env_path = os.environ.get("ARM_CAMERA_CALIBRATION_FILE")
    if env_path:
        return Path(env_path)
    here = Path(__file__).resolve().parent
    shared = here / "lerobot" / "examples" / "phone_to_so100" / CALIBRATION_FILE
    if shared.is_file():
        return shared
    return here / CALIBRATION_FILE


def _nvarguscamerasrc_args() -> list[str]:
    return [
        "nvarguscamerasrc",
        f"sensor-id={CSI_SENSOR_ID}",
        "wbmode=0",
        "awblock=true",
    ]


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
            *_nvarguscamerasrc_args(),
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


def normalize_frame_to_csi_size(frame: np.ndarray | None) -> np.ndarray | None:
    """
    Force BGR frames to exactly CSI_WIDTH x CSI_HEIGHT (same as arm_camera appsink output).

    FFmpeg/MPEG-TS decode often yields padded sizes (e.g. height multiple of 16) or odd SAR;
    we center-crop when larger, resize when smaller, so saved frames and YOLO see true 16:9 640x360.
    """
    if frame is None or frame.size == 0:
        return frame
    h, w = frame.shape[:2]
    tw, th = CSI_WIDTH, CSI_HEIGHT
    if w == tw and h == th:
        return frame
    if w >= tw and h >= th:
        x0 = (w - tw) // 2
        y0 = (h - th) // 2
        return frame[y0 : y0 + th, x0 : x0 + tw].copy()
    return cv2.resize(frame, (tw, th), interpolation=cv2.INTER_AREA)


def _start_csi_tcp_mpegts_stream() -> subprocess.Popen:
    # Same Argus + NVMM NV12 + nvvidconv front-end as arm_camera.py; then explicit I420 WxH + square PAR for x264.
    caps_nvmm = f"video/x-raw(memory:NVMM),width={CSI_WIDTH},height={CSI_HEIGHT},format=NV12,framerate={CSI_FPS}/1"
    caps_i420 = (
        f"video/x-raw,format=I420,width={CSI_WIDTH},height={CSI_HEIGHT},"
        "pixel-aspect-ratio=(fraction)1/1"
    )
    nv_args: list[str] = ["nvvidconv"]
    if CSI_FLIP_METHOD != 0:
        nv_args.append(f"flip-method={CSI_FLIP_METHOD}")

    gst_cmd = [
        SYSTEM_GST_LAUNCH,
        "-q",
        *_nvarguscamerasrc_args(),
        "!",
        caps_nvmm,
        "!",
        *nv_args,
        "!",
        caps_i420,
        "!",
        "x264enc",
        "tune=zerolatency",
        "speed-preset=ultrafast",
        "bitrate=8000",
        "bframes=0",
        "key-int-max=30",
        "byte-stream=true",
        "!",
        "h264parse",
        "config-interval=1",
        "!",
        "mpegtsmux",
        "!",
        "tcpserversink",
        f"host={TCP_STREAM_BIND_HOST}",
        f"port={TCP_PORT}",
        "sync=false",
    ]
    gst_log = open(GST_LOG_PATH, "w", encoding="utf-8")
    gst_env = _build_system_gst_env()
    csi_error = _probe_csi_camera(gst_env)
    if csi_error is not None:
        gst_log.close()
        raise RuntimeError(csi_error)
    proc = subprocess.Popen(gst_cmd, stdout=subprocess.DEVNULL, stderr=gst_log, env=gst_env)
    print(f"[camera] Started gst-launch (log: {GST_LOG_PATH})")
    return proc


def _stop_gst_stream() -> None:
    global _gst_proc
    if _gst_proc is None:
        return
    if _gst_proc.poll() is None:
        _gst_proc.terminate()
        try:
            _gst_proc.wait(timeout=3.0)
        except subprocess.TimeoutExpired:
            _gst_proc.kill()
    _gst_proc = None


def _load_color_matrix() -> np.ndarray | None:
    global _color_matrix, _color_matrix_loaded
    if _color_matrix_loaded:
        return _color_matrix
    _color_matrix_loaded = True
    path = _calibration_json_path()
    if not path.is_file():
        print(f"[camera] No color calibration file at {path}")
        _color_matrix = None
        return None
    try:
        with open(path, encoding="utf-8") as f:
            data = json.load(f)
        _color_matrix = np.array(data, dtype=np.float32)
    except (OSError, json.JSONDecodeError, ValueError) as e:
        print(f"[camera] Invalid color calibration {path}: {e}")
        _color_matrix = None
        return None
    print(f"[camera] Loaded color calibration from {path}")
    return _color_matrix


def apply_color_correction(frame):
    """Linear BGR matrix from color_calibration.json (same as arm_camera.py), if present."""
    if frame is None:
        return None
    M = _load_color_matrix()
    if M is None:
        return frame
    img = frame.astype(np.float32) / 255.0
    img = np.dot(img, M)
    img = np.clip(img, 0, 1)
    return (img * 255).astype(np.uint8)


def initialize_camera(model_path: str | os.PathLike | None = None) -> None:
    """
    Start CSI → H.264/MPEG-TS (system gst-launch), open tcp:// with OpenCV FFmpeg, load YOLO.

    Args:
        model_path: Weights file (.pt). If None, uses DEFAULT_YOLO_MODEL_PATH.
    """
    global _cap, _model, _loaded_model_path, _gst_proc

    mp = _resolve_model_path(model_path)
    if (
        _cap is not None
        and _cap.isOpened()
        and _model is not None
        and _loaded_model_path == mp
        and _gst_proc is not None
        and _gst_proc.poll() is None
    ):
        return

    try:
        need_camera = _cap is None or not _cap.isOpened() or _gst_proc is None or _gst_proc.poll() is not None

        if need_camera:
            if _cap is not None:
                _cap.release()
                _cap = None
            _stop_gst_stream()

            _gst_proc = _start_csi_tcp_mpegts_stream()
            time.sleep(0.5)

            deadline = time.time() + 8.0
            while time.time() < deadline:
                if _gst_proc.poll() is not None:
                    raise RuntimeError(
                        f"gst-launch exited early; see {GST_LOG_PATH} for Argus/encoder errors."
                    )
                _cap = cv2.VideoCapture(
                    f"tcp://{TCP_STREAM_CLIENT_HOST}:{TCP_PORT}", cv2.CAP_FFMPEG
                )
                if _cap.isOpened():
                    break
                if _cap is not None:
                    _cap.release()
                    _cap = None
                time.sleep(0.2)

            if _cap is None or not _cap.isOpened():
                _stop_gst_stream()
                raise RuntimeError(
                    f"Failed to open TCP MPEG-TS stream (FFmpeg). See {GST_LOG_PATH}."
                )

            for _ in range(30):
                _cap.read()
                time.sleep(0.03)

            ok, _ = read_bgr_frame(timeout_sec=10.0)
            if not ok:
                if _cap is not None:
                    _cap.release()
                    _cap = None
                _stop_gst_stream()
                raise RuntimeError(
                    f"No decoded frames from tcp://{TCP_STREAM_CLIENT_HOST}:{TCP_PORT}. "
                    f"See {GST_LOG_PATH}."
                )
            print(f"[camera] Stream OK {CSI_WIDTH}x{CSI_HEIGHT}@{CSI_FPS} (TCP decode)")

        if _model is None or _loaded_model_path != mp:
            _model = YOLO(mp)
            _loaded_model_path = mp
            print(f"[model] Loaded YOLO from {mp}")

        _load_color_matrix()

    except Exception as e:
        print(f"Error initializing camera: {e}")
        raise


def calculate_depth(radius: float) -> float:
    if radius > 0:
        depth = 1716 / radius
        return round(depth, 1)
    return None


def detect_ball(frame) -> tuple[float, float, float] | None:
    if _model is None or frame is None:
        return None

    try:
        frame = apply_color_correction(frame)
        if frame is None:
            return None
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

        pixel_to_cm = 0.1
        x_cm = (cx - frame_w / 2.0) * pixel_to_cm
        z_cm = 0.0

        radius = diameter / 2.0
        depth_cm = calculate_depth(radius)
        y_cm = -depth_cm if depth_cm else None

        if y_cm is None:
            return None

        return (x_cm, y_cm, z_cm)

    except Exception as e:
        print(f"Error in detection: {e}")
        return None


def read_bgr_frame(timeout_sec: float = 2.0) -> tuple[bool, np.ndarray | None]:
    """Read one decoded BGR frame (640x360), retrying briefly; geometry matches arm_camera.py."""
    global _cap
    deadline = time.time() + timeout_sec
    while time.time() < deadline:
        if _cap is None or not _cap.isOpened():
            return False, None
        ret, frame = _cap.read()
        if ret and frame is not None and frame.size > 0:
            return True, normalize_frame_to_csi_size(frame)
        time.sleep(0.02)
    return False, None


def get_video_capture():
    return _cap
