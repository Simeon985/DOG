import os
import json
import subprocess
import sys
from pathlib import Path

# Use the interpreter that launched this script so the child process sees the same environment.
SYSTEM_PYTHON = sys.executable
GST_PLUGIN_SCANNER = "/usr/lib/aarch64-linux-gnu/gstreamer1.0/gstreamer-1.0/gst-plugin-scanner"
GST_PLUGIN_SYSTEM_PATH = "/usr/lib/aarch64-linux-gnu/gstreamer-1.0"
SCRIPT_DIR = Path(__file__).resolve().parent
CALIBRATION_FILE = SCRIPT_DIR / "color_calibration.json"
BALL_DIR = SCRIPT_DIR / "ball"
MODEL_DIR = SCRIPT_DIR.parent / "models"

_CHILD_SCRIPT = r"""
import json
import time
import os
from pathlib import Path

import cv2
import numpy as np
from ultralytics import YOLO

CALIBRATION_FILE = Path(os.environ["ARM_CAMERA_CALIBRATION_FILE"])
MODEL_DIR = Path(os.environ["BALL_MODEL_DIR"])
DETECTION_CONF_THRESHOLD = float(os.environ.get("BALL_DETECTION_CONF_THRESHOLD", "0.25"))

MODEL_SPECS = [
    ("balls_old", MODEL_DIR / "balls_old.pt"),
    ("balls_ourdata", MODEL_DIR / "balls_ourdata.pt"),
    ("balls_ourdata_augmented", MODEL_DIR / "balls_ourdata_augmented.pt"),
]


def csi_pipeline():
    return (
        "nvarguscamerasrc sensor-id=0 wbmode=0 awblock=true ! "
        "video/x-raw(memory:NVMM),width=1280,height=720,framerate=30/1 ! "
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

    try:
        for _ in range(60):
            cap.read()
            time.sleep(0.03)

        ok, frame = cap.read()
        if not ok or frame is None:
            raise RuntimeError("Failed to capture frame")
        return frame
    finally:
        cap.release()


def load_matrix():
    if not CALIBRATION_FILE.exists():
        return None
    with open(CALIBRATION_FILE, "r") as f:
        return np.array(json.load(f), dtype=np.float32)


def apply_matrix(image, matrix):
    img = image.astype(np.float32) / 255.0
    img = np.dot(img, matrix)
    img = np.clip(img, 0, 1)
    return (img * 255).astype(np.uint8)


def load_models():
    models = []
    for name, path in MODEL_SPECS:
        if not path.exists():
            raise FileNotFoundError(f"Missing model file: {path}")
        print(f"Loading {name} from {path}")
        models.append((name, YOLO(str(path))))
    return models


def detect_with_model(model, frame):
    results = model(frame, verbose=False, conf=DETECTION_CONF_THRESHOLD)
    best_conf = 0.0
    best_count = 0
    for result in results:
        for box in result.boxes:
            conf = float(box.conf[0])
            best_count += 1
            if conf > best_conf:
                best_conf = conf
    return best_conf >= DETECTION_CONF_THRESHOLD, best_conf, best_count


def main():
    models = load_models()
    matrix = load_matrix()
    stats = {name: {"detected": 0, "missed": 0, "total": 0} for name, _ in models}
    print("Press ENTER to capture a frame and evaluate all models, or type q then ENTER to quit.")

    while True:
        user_input = input()
        if user_input.strip().lower() in {"q", "quit", "exit"}:
            break

        frame = capture_frame()
        if matrix is not None:
            frame = apply_matrix(frame, matrix)

        print("\nFrame captured. Evaluating models...")
        for name, model in models:
            detected, best_conf, box_count = detect_with_model(model, frame)
            stats[name]["total"] += 1
            if detected:
                stats[name]["detected"] += 1
            else:
                stats[name]["missed"] += 1

            status = "DETECTED" if detected else "missed"
            print(f"{name}: {status} | best_conf={best_conf:.3f} | boxes={box_count}")

        print("Running totals:")
        for name in stats:
            entry = stats[name]
            print(
                f"  {name}: detected={entry['detected']}, missed={entry['missed']}, total={entry['total']}"
            )
        print()


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
    env["ARM_CAMERA_CALIBRATION_FILE"] = str(CALIBRATION_FILE)
    env["BALL_MODEL_DIR"] = str(MODEL_DIR)
    return env



def main() -> int:
    cmd = [SYSTEM_PYTHON, "-c", _CHILD_SCRIPT]
    try:
        completed = subprocess.run(cmd, cwd=SCRIPT_DIR, env=_system_env(), check=False)
    except OSError as exc:
        print(f"Failed to launch system python: {exc}")
        return 1
    return completed.returncode


if __name__ == "__main__":
    raise SystemExit(main())