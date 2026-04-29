import os
import json
import subprocess
import sys
from pathlib import Path

SYSTEM_PYTHON = "/usr/bin/python3"
GST_PLUGIN_SCANNER = "/usr/lib/aarch64-linux-gnu/gstreamer1.0/gstreamer-1.0/gst-plugin-scanner"
GST_PLUGIN_SYSTEM_PATH = "/usr/lib/aarch64-linux-gnu/gstreamer-1.0"
SCRIPT_DIR = Path(__file__).resolve().parent
CALIBRATION_FILE = SCRIPT_DIR / "color_calibration.json"
BALL_DIR = SCRIPT_DIR / "ball"

_CHILD_SCRIPT = r"""
import json
import time
import os
from pathlib import Path

import cv2
import numpy as np

CALIBRATION_FILE = Path(os.environ["ARM_CAMERA_CALIBRATION_FILE"])
BALL_DIR = Path(os.environ["BALL_CAPTURE_OUTPUT_DIR"])


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


def next_output_path(output_dir: Path) -> Path:
    index = 104
    while True:
        candidate = output_dir / f"ball_{index}.png"
        if not candidate.exists():
            return candidate
        index += 1


def main():
    output_dir = BALL_DIR
    output_dir.mkdir(parents=True, exist_ok=True)
    matrix = load_matrix()
    print("Press ENTER to save an image, or type q then ENTER to quit.")

    while True:
        user_input = input()
        if user_input.strip().lower() in {"q", "quit", "exit"}:
            break

        frame = capture_frame()
        if matrix is not None:
            frame = apply_matrix(frame, matrix)
        output_path = next_output_path(output_dir)
        if not cv2.imwrite(str(output_path), frame):
            raise RuntimeError(f"Failed to save image to {output_path}")
        print(output_path.name)


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
    env["BALL_CAPTURE_OUTPUT_DIR"] = str(BALL_DIR)
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
