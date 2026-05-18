import os
import sys
import time
import json
from pathlib import Path

# ----------------------------------------------------------------------
# 1. Set environment ONCE before importing OpenCV/GStreamer
# ----------------------------------------------------------------------
GST_PLUGIN_SCANNER = "/usr/lib/aarch64-linux-gnu/gstreamer1.0/gstreamer-1.0/gst-plugin-scanner"
GST_PLUGIN_SYSTEM_PATH = "/usr/lib/aarch64-linux-gnu/gstreamer-1.0"
CALIBRATION_FILE = Path(__file__).resolve().parent / "color_calibration.json"

# Remove conda/GST pollution that breaks nvarguscamerasrc
os.environ.pop("LD_PRELOAD", None)
os.environ.pop("CONDA_PREFIX", None)
os.environ.pop("CONDA_DEFAULT_ENV", None)
for key in ("GST_PLUGIN_PATH", "GST_PLUGIN_PATH_1_0",
            "GST_PLUGIN_SYSTEM_PATH", "GST_PLUGIN_SYSTEM_PATH_1_0"):
    os.environ.pop(key, None)

os.environ["GST_PLUGIN_SCANNER"] = GST_PLUGIN_SCANNER
os.environ["GST_PLUGIN_SYSTEM_PATH"] = GST_PLUGIN_SYSTEM_PATH
os.environ["GST_PLUGIN_PATH"] = GST_PLUGIN_SYSTEM_PATH
os.environ["LD_LIBRARY_PATH"] = "/usr/lib/aarch64-linux-gnu:/lib/aarch64-linux-gnu:/usr/lib:/lib"

# Now import OpenCV (after environment is clean)
import cv2
import numpy as np

# ----------------------------------------------------------------------
# 2. Persistent camera singleton
# ----------------------------------------------------------------------
_camera = None

def csi_pipeline():
    """GStreamer pipeline string for the CSI camera."""
    return (
        "nvarguscamerasrc sensor-id=0 wbmode=0 awblock=true ! "
        "video/x-raw(memory:NVMM),width=640,height=360,framerate=30/1 ! "
        "nvvidconv ! "
        "video/x-raw,format=BGRx ! "
        "videoconvert ! "
        "video/x-raw,format=BGR ! "
        "appsink emit-signals=false sync=false max-buffers=2 drop=true"
    )

def get_camera():
    """Return the singleton VideoCapture object, opening it if needed."""
    global _camera
    if _camera is None or not _camera.isOpened():
        if _camera is not None:
            _camera.release()
        pipeline = csi_pipeline()
        _camera = cv2.VideoCapture(pipeline, cv2.CAP_GSTREAMER)
        if not _camera.isOpened():
            raise RuntimeError("Failed to open CSI camera. Check no other process uses it.")
        # Drop any stale buffers
        for _ in range(5):
            _camera.grab()
        # Optional: set buffer size (may be ignored by GStreamer, but harmless)
        _camera.set(cv2.CAP_PROP_BUFFERSIZE, 1)
    return _camera

def capture_frame():
    """
    Grab one BGR frame from the persistent camera.
    Retries a few times if the frame is missing (rare after boot).
    """
    cap = get_camera()
    max_attempts = 3
    for attempt in range(max_attempts):
        ret, frame = cap.read()
        if ret and frame is not None and frame.size > 0:
            return frame
        time.sleep(0.03)   # short delay before retry
    raise RuntimeError("Failed to read frame from CSI camera (no data).")

# ----------------------------------------------------------------------
# 3. Calibration and color correction (unchanged logic)
# ----------------------------------------------------------------------
REFERENCE_COLORS = np.array([
    [115, 82, 68], [194,150,130], [98,122,157], [87,108,67],
    [133,128,177], [103,189,170], [214,126,44], [80,91,166],
    [193,90,99], [94,60,108], [157,188,64], [224,163,46],
    [56,61,150], [70,148,73], [175,54,60], [231,199,31],
    [187,86,149], [8,133,161], [243,243,242], [200,200,200],
    [160,160,160], [122,122,121], [85,85,85], [52,52,52]
], dtype=np.float32)

clicked_points = []

def click_event(event, x, y, flags, param):
    global clicked_points
    if event == cv2.EVENT_LBUTTONDOWN:
        clicked_points.append((x, y))
        print(f"Point {len(clicked_points)}: {x}, {y}")

def collect_points(image):
    global clicked_points
    clicked_points = []
    clone = image.copy()
    cv2.imshow("Click 24 color patches", clone)
    cv2.setMouseCallback("Click 24 color patches", click_event)
    print("Click the CENTER of each color patch (24 total)...")
    while len(clicked_points) < 24:
        cv2.imshow("Click 24 color patches", clone)
        cv2.waitKey(1)
    cv2.destroyAllWindows()
    return clicked_points

def compute_matrix(measured, reference):
    measured = measured / 255.0
    reference = reference / 255.0
    M, _, _, _ = np.linalg.lstsq(measured, reference, rcond=None)
    return M

def apply_matrix(image, M):
    img = image.astype(np.float32) / 255.0
    img = np.dot(img, M)
    img = np.clip(img, 0, 1)
    return (img * 255).astype(np.uint8)

def save_matrix(M):
    with open(CALIBRATION_FILE, "w") as f:
        json.dump(M.tolist(), f)

def load_matrix():
    if not CALIBRATION_FILE.exists():
        return None
    with open(CALIBRATION_FILE, "r") as f:
        return np.array(json.load(f), dtype=np.float32)

def calibrate():
    """Interactive calibration – uses persistent camera."""
    frame = capture_frame()
    points = collect_points(frame)
    measured = []
    for (x, y) in points:
        patch = frame[y-5:y+5, x-5:x+5]
        mean_color = patch.mean(axis=(0, 1))
        measured.append(mean_color)
    measured = np.array(measured, dtype=np.float32)
    M = compute_matrix(measured, REFERENCE_COLORS)
    save_matrix(M)
    print("Calibration saved!")
    return M

def capture_corrected(output="image.png"):
    """Capture one frame, apply color correction, save to file."""
    frame = capture_frame()
    M = load_matrix()
    if M is not None:
        frame = apply_matrix(frame, M)
    cv2.imwrite(output, frame)
    print(f"Saved to {output}")

# ----------------------------------------------------------------------
# 4. Public API – direct replacement for the old take_image()
# ----------------------------------------------------------------------
def take_image(path: str | os.PathLike[str]) -> str:
    """Save a corrected frame to the given path (overwrites). Returns the path."""
    output_path = str(Path(path).resolve())
    capture_corrected(output_path)
    return output_path

# ----------------------------------------------------------------------
# 5. Command-line interface (calibrate or capture)
# ----------------------------------------------------------------------
if __name__ == "__main__":
    if len(sys.argv) > 1 and sys.argv[1] == "calibrate":
        calibrate()
    elif len(sys.argv) > 1:
        capture_corrected(sys.argv[1])
    else:
        capture_corrected()