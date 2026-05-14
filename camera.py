"""
Camera module: CSI via system GStreamer (nvarguscamerasrc) → MPEG-TS/TCP → OpenCV FFmpeg → BGR + YOLO.

Uses the same reliable TCP/FFmpeg method that previously worked.
Detection logic (color correction, coordinates from picture) is unchanged.
"""

import json
import os
import subprocess
import time
import ctypes
import math
import atexit
import signal
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
_frame_count: int = 0
_save_debug_frames: bool = False
_debug_frames_dir: Path | None = None

CALIBRATION_FILE = "lerobot/color_calibration.json"
DEFAULT_YOLO_MODEL_PATH = "models/balls_ourdata_augmented.pt"

YOLO_DEVICE = 0 if torch.cuda.is_available() else "cpu"
YOLO_HALF = torch.cuda.is_available()
YOLO_IMGSZ = 224
CONF_THRESHOLD = 0.2
GREEN_CLASS_INDEX = 0
CAMERA_H_FOV_DEG = 60

CSI_SENSOR_ID = 0  # override with env DOG_CSI_SENSOR_ID=1 for the other CSI connector
CSI_FLIP_METHOD = 0
CSI_WIDTH = 640
CSI_HEIGHT = 360
CSI_FPS = 30

# TCP streaming (same as old working version)
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
    # nvarguscamerasrc / NVMM often need EGL; SSH terminals may lack DISPLAY
    if not (env.get("DISPLAY") or "").strip():
        env["DISPLAY"] = ":0"
    xa = Path.home() / ".Xauthority"
    if xa.is_file() and not (env.get("XAUTHORITY") or "").strip():
        env["XAUTHORITY"] = str(xa)
    return env


def _resolve_model_path(model_path: str | os.PathLike | None) -> str:
    base = model_path if model_path is not None else DEFAULT_YOLO_MODEL_PATH
    return str(Path(base).expanduser().resolve())


def _calibration_json_path() -> Path:
    env_path = os.environ.get("ARM_CAMERA_CALIBRATION_FILE")
    if env_path:
        return Path(env_path)
    here = Path(__file__).resolve().parent
    shared = here / "lerobot" / CALIBRATION_FILE
    if shared.is_file():
        return shared
    return here / CALIBRATION_FILE


def _csi_sensor_id() -> int:
    raw = os.environ.get("DOG_CSI_SENSOR_ID", "").strip()
    if not raw:
        return CSI_SENSOR_ID
    try:
        return int(raw)
    except ValueError:
        return CSI_SENSOR_ID


def _nvarguscamerasrc_args() -> list[str]:
    return [
        "nvarguscamerasrc",
        f"sensor-id={_csi_sensor_id()}",
        "wbmode=0",
        "awblock=true",
    ]


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
    # Same Argus + NVMM NV12 + nvvidconv front-end as arm_camera.py; then I420 + square PAR for x264.
    caps_nvmm = (
        f"video/x-raw(memory:NVMM),width={CSI_WIDTH},height={CSI_HEIGHT},format=NV12,framerate={CSI_FPS}/1"
    )
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

    # No probe – it can lock the camera. Just start the pipeline.
    proc = subprocess.Popen(
        gst_cmd,
        stdout=subprocess.DEVNULL,
        stderr=gst_log,
        env=gst_env,
        start_new_session=True,
    )
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


def _cleanup_all() -> None:
    """Clean up all resources on program exit."""
    global _cap
    try:
        if _cap is not None and _cap.isOpened():
            _cap.release()
    except Exception:
        pass
    _stop_gst_stream()


# Register cleanup on program exit
atexit.register(_cleanup_all)


def _signal_handler(signum, frame):
    """Handle termination signals gracefully."""
    _cleanup_all()
    exit(0)


# Register signal handlers for Ctrl+C and kill signals
signal.signal(signal.SIGINT, _signal_handler)
signal.signal(signal.SIGTERM, _signal_handler)


def enable_debug_frame_saving(enable: bool = True, max_frames: int = 50) -> None:
    """Enable or disable saving debug frames to disk."""
    global _save_debug_frames, _debug_frames_dir, _frame_count
    _save_debug_frames = enable
    _frame_count = 0
    if enable:
        _debug_frames_dir = Path("debug/camera_frames")
        _debug_frames_dir.mkdir(parents=True, exist_ok=True)
        # Clear old frames
        for f in _debug_frames_dir.glob("*.png"):
            f.unlink()
        print(f"[camera] Debug frame saving enabled. Saving up to {max_frames} frames to {_debug_frames_dir}")


def _save_debug_frame(frame: np.ndarray, label: str = "") -> None:
    """Save a frame to the debug directory."""
    global _frame_count, _save_debug_frames
    if not _save_debug_frames or _debug_frames_dir is None:
        return
    if _frame_count >= 50:
        return
    if frame is None or frame.size == 0:
        return
    try:
        filename = _debug_frames_dir / f"frame_{_frame_count:03d}_{label}.png"
        cv2.imwrite(str(filename), frame)
        print(f"[camera] Saved {filename}")
        _frame_count += 1
    except Exception as e:
        print(f"[camera] Failed to save debug frame: {e}")


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
            # Give Argus and the pipeline time to initialise and bind the TCP port
            time.sleep(2.0)

            # Wait for the TCP port to become listening (up to 8 seconds)
            deadline = time.time() + 8.0
            port_open = False
            while time.time() < deadline:
                if _gst_proc.poll() is not None:
                    # gst-launch died – read its stderr from the log file
                    with open(GST_LOG_PATH, "r", encoding="utf-8") as logf:
                        log_content = logf.read()
                    raise RuntimeError(
                        f"gst-launch exited early. Log:\n{log_content}"
                    )
                # Simple check: try to connect to the port using a dummy socket
                import socket
                s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                s.settimeout(0.2)
                try:
                    s.connect((TCP_STREAM_CLIENT_HOST, TCP_PORT))
                    s.close()
                    port_open = True
                    break
                except (ConnectionRefusedError, socket.timeout):
                    pass
                time.sleep(0.3)

            if not port_open:
                _stop_gst_stream()
                raise RuntimeError(
                    f"TCP port {TCP_PORT} never became listening. See {GST_LOG_PATH} for gst-launch errors."
                )

            # Now open the stream with OpenCV
            _cap = cv2.VideoCapture(
                f"tcp://{TCP_STREAM_CLIENT_HOST}:{TCP_PORT}", cv2.CAP_FFMPEG
            )
            if not _cap.isOpened():
                _stop_gst_stream()
                raise RuntimeError(
                    f"Failed to open TCP MPEG-TS stream (FFmpeg). See {GST_LOG_PATH}."
                )

            # Drain a few frames to stabilise
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


def detect_ball(frame, zoek_in_lucht: bool) -> tuple[float, float, float] | None:
    """
    cam_angle = vertical camera angle (0 = parallel to floor, positive = upwards).
    When searching in air: distance_cam_to_middle = 24 cm, cam_angle = 18°.
    When searching on ground: distance_cam_to_middle = 24 cm, cam_angle = -30°.
    """
    if _model is None or frame is None:
        return None

    try:
        if _save_debug_frames:
            _save_debug_frame(frame, "input")

        if zoek_in_lucht:
            distance_cam_to_middle, cam_angle = 24, math.radians(18)
        else:
            distance_cam_to_middle, cam_angle = 24, math.radians(-30)

        frame = apply_color_correction(frame)
        if frame is None:
            return None

        if _save_debug_frames:
            _save_debug_frame(frame, "color_corrected")

        # Lazy import to avoid circular import at module import time.
        from coordinates_from_picture import get_coordinates_from_frame

        x_cam, y_cam, z_cam = get_coordinates_from_frame(frame)
        # Transform camera coordinates to robot coordinates
        x = x_cam
        z = math.sin(cam_angle) * abs(z_cam) + math.cos(cam_angle) * y_cam
        y = math.cos(cam_angle) * abs(z_cam) - math.sin(cam_angle) * y_cam - distance_cam_to_middle
        print("x,y,z")
        print(x, y, z)
        return x, y, z

    except Exception as e:
        print(f"Error in detection: {e}")
        return None


def read_bgr_frame(timeout_sec: float = 2.0) -> tuple[bool, np.ndarray | None]:
    """Read one decoded BGR frame (640x360), retrying briefly; geometry matches arm_camera.py."""
    global _cap
    deadline = time.time() + timeout_sec
    print(f"[camera] read_bgr_frame: checking _cap (exists={_cap is not None}, opened={_cap.isOpened() if _cap is not None else 'N/A'}) timeout={timeout_sec}")
    while time.time() < deadline:
        if _cap is None or not _cap.isOpened():
            print("[camera] read_bgr_frame: _cap is None or not opened — attempting to reinitialize camera")
            try:
                initialize_camera()
            except Exception as e:
                print(f"[camera] read_bgr_frame: reinitialize failed: {e}")
                # wait briefly and continue retrying until deadline
                time.sleep(0.05)
                continue
            # after attempting reinit, re-check _cap
            print(f"[camera] read_bgr_frame: reinitialized, cap exists={_cap is not None}, opened={_cap.isOpened() if _cap is not None else 'N/A'}")
            if _cap is None or not _cap.isOpened():
                time.sleep(0.05)
                continue
        ret, frame = _cap.read()
        if ret and frame is not None and frame.size > 0:
            frame = normalize_frame_to_csi_size(frame)
            if _save_debug_frames:
                _save_debug_frame(frame, "raw")
            return True, frame
        time.sleep(0.02)
    print("[camera] read_bgr_frame: timed out waiting for frame")
    return False, None


def get_video_capture():
    return _cap