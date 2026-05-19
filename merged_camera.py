#!/usr/bin/env python3
"""
Merged Camera Module for NVIDIA Jetson (CSI camera)

Uses direct GStreamer pipeline inside OpenCV (CAP_GSTREAMER).
Provides:
- initialize_camera() – opens camera, loads YOLO model, color matrix
- read_bgr_frame() – reads a single frame (with auto‑reinit on failure)
- detect_ball() – runs YOLO + coordinate geometry (from File 1)
- take_image() – saves a color‑corrected image (from File 2)
- calibrate() – interactive color calibration (from File 2)
- get_video_capture() – returns the current VideoCapture object (compatibility)
- apply_color_correction() – applies loaded color matrix (public API)

All resources are properly released, and a short delay is added before closing
to let the Jetson camera driver fully reset the sensor.
"""

import json
import os
import sys
import time
import math
import signal
import atexit
from pathlib import Path

import cv2
import numpy as np
import torch
from ultralytics import YOLO

# ----------------------------------------------------------------------
# 1. Environment sanitisation (avoid conda/gstreamer conflicts)
# ----------------------------------------------------------------------
def _sanitise_environment():
    """Remove environment variables that break nvarguscamerasrc."""
    for key in ("LD_PRELOAD", "CONDA_PREFIX", "CONDA_DEFAULT_ENV",
                "GST_PLUGIN_PATH", "GST_PLUGIN_PATH_1_0",
                "GST_PLUGIN_SYSTEM_PATH", "GST_PLUGIN_SYSTEM_PATH_1_0"):
        os.environ.pop(key, None)

    os.environ["GST_PLUGIN_SCANNER"] = "/usr/lib/aarch64-linux-gnu/gstreamer1.0/gstreamer-1.0/gst-plugin-scanner"
    os.environ["GST_PLUGIN_SYSTEM_PATH"] = "/usr/lib/aarch64-linux-gnu/gstreamer-1.0"
    os.environ["GST_PLUGIN_PATH"] = "/usr/lib/aarch64-linux-gnu/gstreamer-1.0"
    os.environ["LD_LIBRARY_PATH"] = "/usr/lib/aarch64-linux-gnu:/lib/aarch64-linux-gnu:/usr/lib:/lib"

_sanitise_environment()

# ----------------------------------------------------------------------
# 2. Constants (from both files)
# ----------------------------------------------------------------------
CSI_SENSOR_ID = 0          # override with DOG_CSI_SENSOR_ID env var
CSI_WIDTH = 640
CSI_HEIGHT = 360
CSI_FPS = 30
GREEN_CLASS_INDEX = 0
CONF_THRESHOLD = 0.2
YOLO_IMGSZ = 224
CAMERA_H_FOV_DEG = 60

DEFAULT_YOLO_MODEL_PATH = "models/balls_ourdata_augmented.pt"
CALIBRATION_FILE = Path(__file__).resolve().parent / "lerobot" / "color_calibration.json"

# ----------------------------------------------------------------------
# 3. Global state (singleton)
# ----------------------------------------------------------------------
_cap = None                # VideoCapture object
_model = None
_loaded_model_path = None
_color_matrix = None
_color_matrix_loaded = False

# ----------------------------------------------------------------------
# 4. GStreamer pipeline builder
# ----------------------------------------------------------------------
def _csi_pipeline() -> str:
    """Return GStreamer pipeline string for direct OpenCV capture."""
    sensor_id = int(os.environ.get("DOG_CSI_SENSOR_ID", CSI_SENSOR_ID))
    return (
        f"nvarguscamerasrc sensor-id={sensor_id} wbmode=0 awblock=true ! "
        f"video/x-raw(memory:NVMM),width={CSI_WIDTH},height={CSI_HEIGHT},framerate={CSI_FPS}/1 ! "
        "nvvidconv ! "
        "video/x-raw,format=BGRx ! "
        "videoconvert ! "
        "video/x-raw,format=BGR ! "
        "appsink emit-signals=false sync=false max-buffers=2 drop=true"
    )

# ----------------------------------------------------------------------
# 5. Camera initialisation and cleanup
# ----------------------------------------------------------------------
def _release_camera():
    """Release the camera and give the driver time to reset."""
    global _cap
    if _cap is not None:
        _cap.release()
        _cap = None
    # Critical delay: allows Jetson CSI driver to fully release the sensor
    time.sleep(1.5)

def initialize_camera(model_path: str | Path | None = None):
    """
    Open the CSI camera, load YOLO model, and load colour calibration.
    Can be called multiple times – it will reuse existing resources if possible.
    """
    global _cap, _model, _loaded_model_path, _color_matrix, _color_matrix_loaded

    # Resolve model path
    if model_path is None:
        model_path = DEFAULT_YOLO_MODEL_PATH
    model_path = str(Path(model_path).expanduser().resolve())

    # If everything is already initialised and camera is open, do nothing
    if (_cap is not None and _cap.isOpened() and
        _model is not None and _loaded_model_path == model_path):
        return

    # Release any previous camera
    _release_camera()

    # Create new pipeline and open camera
    pipeline = _csi_pipeline()
    _cap = cv2.VideoCapture(pipeline, cv2.CAP_GSTREAMER)
    if not _cap.isOpened():
        raise RuntimeError("Failed to open CSI camera. Is another process using it?")

    # Drop stale buffers
    for _ in range(5):
        _cap.grab()
    # Optionally reduce buffer size (may be ignored, but harmless)
    _cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)

    # Test read a few frames
    for _ in range(10):
        ret, frame = _cap.read()
        if ret and frame is not None:
            break
        time.sleep(0.03)
    else:
        _release_camera()
        raise RuntimeError("Camera opened but no data received. Check sensor connection.")

    # Load YOLO model if needed
    if _model is None or _loaded_model_path != model_path:
        _model = YOLO(model_path)
        _loaded_model_path = model_path
        print(f"[YOLO] Loaded model from {model_path}")

    # Load colour calibration matrix (if exists)
    if not _color_matrix_loaded:
        _color_matrix = _load_color_matrix()
        _color_matrix_loaded = True

    print("[Camera] Initialised successfully")

def get_video_capture():
    """
    Return the current VideoCapture object (for compatibility with old code).
    May be None if camera not initialised.
    """
    return _cap

def _load_color_matrix() -> np.ndarray | None:
    """Load 3x3 colour correction matrix from JSON file."""
    path = CALIBRATION_FILE
    if not path.is_file():
        print(f"[Colour] No calibration file at {path}")
        return None
    try:
        with open(path, encoding="utf-8") as f:
            data = json.load(f)
        M = np.array(data, dtype=np.float32)
        print(f"[Colour] Loaded calibration from {path}")
        return M
    except Exception as e:
        print(f"[Colour] Failed to load {path}: {e}")
        return None

def apply_color_correction(frame: np.ndarray) -> np.ndarray:
    """Apply 3x3 colour correction matrix if available."""
    if frame is None or _color_matrix is None:
        return frame
    img = frame.astype(np.float32) / 255.0
    img = np.dot(img, _color_matrix)
    img = np.clip(img, 0, 1)
    return (img * 255).astype(np.uint8)

def read_bgr_frame(timeout_sec: float = 2.0) -> tuple[bool, np.ndarray | None]:
    """
    Read one BGR frame (640x360) from the camera.
    If camera fails, attempts to reinitialise once.
    Returns (success, frame).
    """
    global _cap
    deadline = time.time() + timeout_sec
    while time.time() < deadline:
        if _cap is None or not _cap.isOpened():
            print("[Camera] Camera not open, reinitialising...")
            try:
                initialize_camera()
            except Exception as e:
                print(f"[Camera] Reinit failed: {e}")
                time.sleep(0.1)
                continue
        ret, frame = _cap.read()
        if ret and frame is not None and frame.size > 0:
            # Ensure exact dimensions (GStreamer should already give 640x360)
            if frame.shape[1] != CSI_WIDTH or frame.shape[0] != CSI_HEIGHT:
                frame = cv2.resize(frame, (CSI_WIDTH, CSI_HEIGHT))
            return True, frame
        time.sleep(0.02)
    return False, None

# ----------------------------------------------------------------------
# 6. Detection and geometry (from File 1)
# ----------------------------------------------------------------------
def calculate_depth(radius_px: float) -> float | None:
    """Convert pixel radius to depth (cm) using known focal length."""
    if radius_px > 0:
        depth = 1716 / radius_px   # constant from original calibration
        return round(depth, 1)
    return None

def detect_ball(frame: np.ndarray, zoek_in_lucht: bool) -> tuple[float, float, float] | None:
    """
    Run YOLO detection, then convert image coordinates to 3D world coordinates.
    - zoek_in_lucht: True = ball in air (camera tilted up), False = on ground (tilted down)
    Returns (x, y, z) in cm, or None if no ball detected or coordinates invalid.
    """
    if _model is None or frame is None:
        return None

    # Apply colour correction before detection
    frame_corrected = frame
    if frame_corrected is None:
        return None

    # Get raw camera coordinates (may be None if no detection)
    try:
        from coordinates_from_picture import get_coordinates_from_frame
        x_cam, y_cam, z_cam = get_coordinates_from_frame(frame_corrected)
    except ImportError:
        print("[Detection] coordinates_from_picture not found")
        return None
    except Exception as e:
        print(f"[Detection] Error in get_coordinates_from_frame: {e}")
        return None

    # Check that we got valid numbers (not None)
    if not all(isinstance(v, (int, float)) for v in (x_cam, y_cam, z_cam)):
        print("[Detection] Invalid coordinates from get_coordinates_from_frame (got None)")
        return None

    print(f"[Detection] Raw cam coords: ({x_cam:.2f}, {y_cam:.2f}, {z_cam:.2f})")

    # Geometry constants from original script
    if zoek_in_lucht:
        distance_cam_to_middle = 24.0
        cam_angle_rad = math.radians(18)
    else:
        distance_cam_to_middle = 24.0
        cam_angle_rad = math.radians(-30)

    # Transform to world coordinates
    x = float(x_cam)
    z_world = math.sin(cam_angle_rad) * abs(z_cam) + math.cos(cam_angle_rad) * y_cam
    y_world = math.cos(cam_angle_rad) * abs(z_cam) - math.sin(cam_angle_rad) * y_cam + distance_cam_to_middle

    # Final safety: if any result is NaN or infinite, return None
    if not all(math.isfinite(v) for v in (x, y_world, z_world)):
        print("[Detection] Invalid world coordinates (NaN or inf)")
        return None

    print(f"[Detection] World coords: ({x:.2f}, {y_world:.2f}, {z_world:.2f})")
    return (x, y_world, z_world)

# ----------------------------------------------------------------------
# 7. Calibration and still image capture (from File 2)
# ----------------------------------------------------------------------
REFERENCE_COLORS = np.array([
    [115, 82, 68], [194,150,130], [98,122,157], [87,108,67],
    [133,128,177], [103,189,170], [214,126,44], [80,91,166],
    [193,90,99], [94,60,108], [157,188,64], [224,163,46],
    [56,61,150], [70,148,73], [175,54,60], [231,199,31],
    [187,86,149], [8,133,161], [243,243,242], [200,200,200],
    [160,160,160], [122,122,121], [85,85,85], [52,52,52]
], dtype=np.float32)

_clicked_points = []

def _click_event(event, x, y, flags, param):
    if event == cv2.EVENT_LBUTTONDOWN:
        _clicked_points.append((x, y))
        print(f"Point {len(_clicked_points)}: ({x}, {y})")

def calibrate():
    """
    Interactive colour calibration.
    Captures one frame, asks user to click 24 colour patches,
    computes and saves a 3x3 correction matrix.
    """
    initialize_camera()
    ret, frame = read_bgr_frame(timeout_sec=5.0)
    if not ret:
        raise RuntimeError("Could not capture frame for calibration")

    global _clicked_points
    _clicked_points = []
    clone = frame.copy()
    cv2.imshow("Click 24 colour patches", clone)
    cv2.setMouseCallback("Click 24 colour patches", _click_event)
    print("Click the CENTER of each colour patch (24 total)...")
    while len(_clicked_points) < 24:
        cv2.imshow("Click 24 colour patches", clone)
        if cv2.waitKey(1) & 0xFF == 27:  # ESC to cancel
            cv2.destroyAllWindows()
            raise RuntimeError("Calibration cancelled")
    cv2.destroyAllWindows()

    measured = []
    for (x, y) in _clicked_points:
        patch = frame[y-5:y+5, x-5:x+5]
        if patch.size == 0:
            continue
        mean_color = patch.mean(axis=(0, 1))
        measured.append(mean_color)

    if len(measured) != 24:
        print(f"Only {len(measured)} valid patches, aborting.")
        return

    measured = np.array(measured, dtype=np.float32) / 255.0
    reference = REFERENCE_COLORS / 255.0
    M, _, _, _ = np.linalg.lstsq(measured, reference, rcond=None)
    # Save matrix
    CALIBRATION_FILE.parent.mkdir(parents=True, exist_ok=True)
    with open(CALIBRATION_FILE, "w") as f:
        json.dump(M.tolist(), f)
    print(f"Colour calibration saved to {CALIBRATION_FILE}")

def take_image(path: str | Path) -> str:
    """
    Capture a single frame, apply colour correction, save as image.
    Returns the absolute path of the saved file.
    """
    initialize_camera()
    ret, frame = read_bgr_frame(timeout_sec=5.0)
    if not ret:
        raise RuntimeError("Could not capture frame for image")
    frame = apply_color_correction(frame)
    out_path = Path(path).expanduser().resolve()
    out_path.parent.mkdir(parents=True, exist_ok=True)
    cv2.imwrite(str(out_path), frame)
    print(f"Image saved to {out_path}")
    return str(out_path)

# ----------------------------------------------------------------------
# 8. Cleanup on exit
# ----------------------------------------------------------------------
def _cleanup_all():
    _release_camera()

atexit.register(_cleanup_all)
signal.signal(signal.SIGINT, lambda s, f: _cleanup_all() or exit(0))
signal.signal(signal.SIGTERM, lambda s, f: _cleanup_all() or exit(0))

# ----------------------------------------------------------------------
# 9. Command‑line interface
# ----------------------------------------------------------------------
if __name__ == "__main__":
    import argparse
    parser = argparse.ArgumentParser(description="Jetson CSI Camera Tool")
    parser.add_argument("--calibrate", action="store_true", help="Run colour calibration")
    parser.add_argument("--capture", type=str, metavar="FILE", help="Save a colour-corrected image")
    parser.add_argument("--detect", action="store_true", help="Run continuous YOLO detection (air mode)")
    args = parser.parse_args()

    if args.calibrate:
        calibrate()
    elif args.capture:
        take_image(args.capture)
    elif args.detect:
        initialize_camera()
        print("Press Ctrl+C to stop detection (zoek_in_lucht=True)")
        while True:
            ok, frame = read_bgr_frame()
            if not ok:
                print("Failed to read frame, retrying...")
                continue
            result = detect_ball(frame, zoek_in_lucht=True)
            if result:
                x, y, z = result
                print(f"Ball detected: x={x:.1f} cm, y={y:.1f} cm, z={z:.1f} cm")
            else:
                print("No ball detected")
            time.sleep(0.05)
    else:
        # Default: show live preview with colour correction
        initialize_camera()
        print("Live preview (press ESC to exit)")
        while True:
            ok, frame = read_bgr_frame()
            if not ok:
                continue
            frame = apply_color_correction(frame)
            cv2.imshow("CSI Camera", frame)
            if cv2.waitKey(1) & 0xFF == 27:
                break
        cv2.destroyAllWindows()