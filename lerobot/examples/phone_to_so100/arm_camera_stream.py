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
    sensor_id = int(os.environ.get("ARM_CAMERA_SENSOR_ID", "0"))
    return (
        f"nvarguscamerasrc sensor-id={sensor_id} wbmode=0 awblock=true ! "
        "video/x-raw(memory:NVMM),width=1280,height=720,framerate=30/1 ! "
        "nvvidconv ! "
        "video/x-raw,format=BGRx ! "
        "videoconvert ! "
        "video/x-raw,format=BGR ! "
        "appsink drop=true max-buffers=1 sync=false"
    )

def _open_capture():
    cap = cv2.VideoCapture(csi_pipeline(), cv2.CAP_GSTREAMER)
    if not cap.isOpened():
        cap.release()
        raise RuntimeError("Camera failed to open (GStreamer pipeline did not start).")
    return cap


def capture_frame():
    cap = _open_capture()

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

def stream(output="image.png"):
    cap = _open_capture()

    # Let auto exposure/white-balance settle a bit.
    for _ in range(30):
        cap.read()
        time.sleep(0.03)

    M = load_matrix()
    win = "Arm camera (q/ESC quit, s save, r reload calib, c calibrate)"
    if not os.environ.get("DISPLAY") and not os.environ.get("WAYLAND_DISPLAY"):
        cap.release()
        raise RuntimeError(
            "No GUI display found (DISPLAY/WAYLAND_DISPLAY not set). "
            "Run this from a desktop session (or with X11 forwarding) to show the camera window."
        )
    try:
        cv2.namedWindow(win, cv2.WINDOW_NORMAL)
    except cv2.error as exc:
        cap.release()
        raise RuntimeError(
            "OpenCV failed to initialize a GUI window backend. "
            "This usually means you're running headless or missing GTK/Qt support."
        ) from exc

    try:
        while True:
            ret, frame = cap.read()
            if not ret:
                # Argus/GStreamer can occasionally stall; retry a bit, then reopen.
                recovered = False
                for _ in range(10):
                    time.sleep(0.03)
                    ret, frame = cap.read()
                    if ret:
                        recovered = True
                        break
                if not recovered:
                    cap.release()
                    time.sleep(0.2)
                    cap = _open_capture()
                    # Warm up again after reopen
                    for _ in range(10):
                        cap.read()
                        time.sleep(0.03)
                    ret, frame = cap.read()
                    if not ret:
                        raise RuntimeError(
                            "Failed to read frame (even after reopening capture). "
                            "Try rebooting, or set ARM_CAMERA_SENSOR_ID=1 if you're using the other CSI port."
                        )

            if M is not None:
                frame = apply_matrix(frame, M)

            cv2.imshow(win, frame)
            key = cv2.waitKey(1) & 0xFF

            # Quit
            if key in (ord("q"), 27):
                break

            # Save snapshot
            if key == ord("s"):
                cv2.imwrite(output, frame)
                print(f"Saved to {output}")

            # Reload calibration file on demand
            if key == ord("r"):
                M = load_matrix()
                print("Reloaded calibration." if M is not None else "No calibration file found.")

            # Run calibration using the *current raw* frame
            if key == ord("c"):
                # Calibrate on a fresh raw frame (not corrected), then reload.
                calibrate()
                M = load_matrix()
    finally:
        cap.release()
        cv2.destroyAllWindows()


def main():
    if len(sys.argv) > 1 and sys.argv[1] == "calibrate":
        calibrate()
    elif len(sys.argv) > 1 and sys.argv[1] == "snap":
        capture_corrected()
    else:
        stream()


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


def _run_child() -> int:
    cmd = [SYSTEM_PYTHON, "-c", _CHILD_SCRIPT, *sys.argv[1:]]
    try:
        completed = subprocess.run(cmd, env=_system_env(), check=False)
    except OSError as exc:
        print(f"Failed to launch system python: {exc}")
        return 1
    return completed.returncode


if __name__ == "__main__":
    raise SystemExit(_run_child())