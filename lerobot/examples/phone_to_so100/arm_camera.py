import os
import subprocess
import sys
from pathlib import Path

SYSTEM_PYTHON = "/usr/bin/python3"
GST_PLUGIN_SCANNER = "/usr/lib/aarch64-linux-gnu/gstreamer1.0/gstreamer-1.0/gst-plugin-scanner"
GST_PLUGIN_SYSTEM_PATH = "/usr/lib/aarch64-linux-gnu/gstreamer-1.0"
CALIBRATION_FILE = "color_calibration.json"
SCRIPT_DIR = Path(__file__).resolve().parent

_CHILD_SCRIPT = r"""
import json
import os
import sys
import time
from pathlib import Path

import cv2
import numpy as np

CALIBRATION_FILE = Path(os.environ["ARM_CAMERA_CALIBRATION_FILE"])

def csi_pipeline():
    return (
        "nvarguscamerasrc sensor-id=0 wbmode=0 awblock=true ! "
        "video/x-raw(memory:NVMM),width=640,height=360,framerate=30/1 ! "
        "nvvidconv ! "
        "video/x-raw,format=BGRx ! "
        "videoconvert ! "
        "video/x-raw,format=BGR ! "
        "appsink drop=true"
    )


def capture_frame():
    cap = cv2.VideoCapture(csi_pipeline(), cv2.CAP_GSTREAMER)

    if not cap.isOpened():
        raise RuntimeError("Camera failed to open")

    for _ in range(60):
        cap.read()
        time.sleep(0.03)

    ret, frame = cap.read()
    cap.release()

    if not ret:
        raise RuntimeError("Failed to capture frame")

    return frame


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


REFERENCE_COLORS = np.array([
    [115, 82, 68], [194,150,130], [98,122,157], [87,108,67],
    [133,128,177], [103,189,170], [214,126,44], [80,91,166],
    [193,90,99], [94,60,108], [157,188,64], [224,163,46],
    [56,61,150], [70,148,73], [175,54,60], [231,199,31],
    [187,86,149], [8,133,161], [243,243,242], [200,200,200],
    [160,160,160], [122,122,121], [85,85,85], [52,52,52]
], dtype=np.float32)


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
    frame = capture_frame()

    M = load_matrix()
    if M is not None:
        frame = apply_matrix(frame, M)

    cv2.imwrite(output, frame)
    print(f"Saved to {output}")


def main():
    if len(sys.argv) > 1 and sys.argv[1] == "calibrate":
        calibrate()
    else:
        capture_corrected()


if __name__ == "__main__":
    main()
"""


def _system_env() -> dict:
    env = os.environ.copy()
    env.pop("LD_PRELOAD", None)
    env.pop("CONDA_PREFIX", None)
    env.pop("CONDA_DEFAULT_ENV", None)
    env["GST_PLUGIN_SCANNER"] = GST_PLUGIN_SCANNER
    env["GST_PLUGIN_SYSTEM_PATH"] = GST_PLUGIN_SYSTEM_PATH
    env["LD_LIBRARY_PATH"] = "/usr/lib/aarch64-linux-gnu:/lib/aarch64-linux-gnu:/usr/lib:/lib"
    env["ARM_CAMERA_CALIBRATION_FILE"] = str(SCRIPT_DIR / CALIBRATION_FILE)
    return env


def _run_child(args: list[str]) -> int:
    cmd = [SYSTEM_PYTHON, "-c", _CHILD_SCRIPT, *args]
    try:
        completed = subprocess.run(cmd, env=_system_env(), check=False)
    except OSError as exc:
        print(f"Failed to launch system python: {exc}")
        return 1
    return completed.returncode


def take_image(path: str | os.PathLike[str]) -> str:
    output_path = str(Path(path).resolve())
    exit_code = _run_child([output_path])
    if exit_code != 0:
        raise RuntimeError(f"Failed to capture image to {output_path}")
    return output_path


if __name__ == "__main__":
    raise SystemExit(_run_child(sys.argv[1:]))