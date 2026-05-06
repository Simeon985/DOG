import os
import subprocess
import sys
from pathlib import Path
from typing import Optional
import select
import time
import tempfile

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
        "video/x-raw(memory:NVMM),width=600,height=400,framerate=30/1 ! "
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

_CHILD_IPC_SCRIPT = r"""
import json
import os
import struct
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


def load_matrix():
    if not CALIBRATION_FILE.exists():
        return None
    with open(CALIBRATION_FILE, "r") as f:
        return np.array(json.load(f), dtype=np.float32)


def apply_matrix(image, M):
    img = image.astype(np.float32) / 255.0
    img = np.dot(img, M)
    img = np.clip(img, 0, 1)
    return (img * 255).astype(np.uint8)


def main():
    cap = _open_capture()

    # Warmup for AWB/AE settle.
    for _ in range(30):
        cap.read()
        time.sleep(0.02)

    M = load_matrix()

    def _reopen_capture():
        nonlocal cap
        try:
            cap.release()
        except Exception:
            pass
        time.sleep(0.2)
        cap = _open_capture()
        # Quick warmup after reopen.
        for _ in range(10):
            cap.read()
            time.sleep(0.02)

    try:
        while True:
            line = sys.stdin.buffer.readline()
            if not line:
                break
            cmd = line.strip().upper()
            if cmd == b"QUIT":
                break
            if cmd == b"RELOAD":
                M = load_matrix()
                continue
            if cmd != b"GET":
                continue

            payload = b""
            frame = None

            # Argus/GStreamer can stall; retry a few reads, then reopen capture.
            for attempt in range(1, 21):
                ret, frame = cap.read()
                if ret and frame is not None:
                    break
                time.sleep(0.02)
                if attempt in (10, 15):
                    try:
                        print("Frame read failed; reopening capture...", file=sys.stderr, flush=True)
                    except Exception:
                        pass
                    try:
                        _reopen_capture()
                    except Exception as exc:
                        try:
                            print(f"Reopen failed: {exc}", file=sys.stderr, flush=True)
                        except Exception:
                            pass

            if frame is not None:
                try:
                    if M is not None:
                        frame = apply_matrix(frame, M)
                    ok, jpg = cv2.imencode(".jpg", frame, [int(cv2.IMWRITE_JPEG_QUALITY), 85])
                    payload = bytes(jpg) if ok else b""
                except Exception as exc:
                    try:
                        print(f"Encode failed: {exc}", file=sys.stderr, flush=True)
                    except Exception:
                        pass
            else:
                try:
                    print(
                        "No frame received after retries; returning empty payload.",
                        file=sys.stderr,
                        flush=True,
                    )
                except Exception:
                    pass

            sys.stdout.buffer.write(struct.pack(">I", len(payload)))
            if payload:
                sys.stdout.buffer.write(payload)
            sys.stdout.buffer.flush()
    finally:
        cap.release()


if __name__ == "__main__":
    main()
"""

_CHILD_DAEMON_SCRIPT = r"""
import json
import os
import time
from pathlib import Path

import cv2
import numpy as np

CALIBRATION_FILE = Path(os.environ["ARM_CAMERA_CALIBRATION_FILE"])
OUT_PATH = Path(os.environ["ARM_CAMERA_OUT_PATH"])
OUT_TMP = OUT_PATH.with_suffix(OUT_PATH.suffix + ".tmp")


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


def load_matrix():
    if not CALIBRATION_FILE.exists():
        return None
    with open(CALIBRATION_FILE, "r") as f:
        return np.array(json.load(f), dtype=np.float32)


def apply_matrix(image, M):
    img = image.astype(np.float32) / 255.0
    img = np.dot(img, M)
    img = np.clip(img, 0, 1)
    return (img * 255).astype(np.uint8)


def main():
    cap = _open_capture()

    # Warmup for AWB/AE settle.
    for _ in range(30):
        cap.read()
        time.sleep(0.02)

    M = load_matrix()
    calib_mtime = CALIBRATION_FILE.stat().st_mtime if CALIBRATION_FILE.exists() else None

    def reopen():
        nonlocal cap
        try:
            cap.release()
        except Exception:
            pass
        time.sleep(0.2)
        cap = _open_capture()
        for _ in range(10):
            cap.read()
            time.sleep(0.02)

    try:
        while True:
            # Reload calibration if file changed.
            try:
                if CALIBRATION_FILE.exists():
                    m = CALIBRATION_FILE.stat().st_mtime
                    if calib_mtime is None or m != calib_mtime:
                        M = load_matrix()
                        calib_mtime = m
            except Exception:
                pass

            ret, frame = cap.read()
            if not ret or frame is None:
                # Try to recover.
                time.sleep(0.02)
                ret, frame = cap.read()
                if not ret or frame is None:
                    reopen()
                    continue

            if M is not None:
                frame = apply_matrix(frame, M)

            ok, jpg = cv2.imencode(".jpg", frame, [int(cv2.IMWRITE_JPEG_QUALITY), 85])
            if ok:
                OUT_TMP.write_bytes(bytes(jpg))
                OUT_TMP.replace(OUT_PATH)

            # ~30 FPS cap, but let CPU breathe.
            time.sleep(0.01)
    finally:
        cap.release()


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


class ArmCameraFrameClient:
    """
    Persistent CSI camera reader using system Python + GStreamer/Argus.

    This avoids repeatedly starting nvargus (slow) and avoids writing image files.
    """

    def __init__(self):
        self._proc: Optional[subprocess.Popen] = None
        self._out_path: Optional[Path] = None
        self._last_mtime: float = 0.0

    def start(self) -> None:
        if self._proc is not None and self._proc.poll() is None:
            return
        out_path = Path(tempfile.gettempdir()) / "arm_latest.jpg"
        env = _system_env()
        env["ARM_CAMERA_OUT_PATH"] = str(out_path)
        cmd = [SYSTEM_PYTHON, "-u", "-c", _CHILD_DAEMON_SCRIPT]
        self._proc = subprocess.Popen(
            cmd,
            env=env,
            stdin=subprocess.DEVNULL,
            stdout=subprocess.DEVNULL,
            stderr=subprocess.PIPE,
        )
        self._out_path = out_path
        self._last_mtime = 0.0

    def close(self) -> None:
        if self._proc is None:
            return
        try:
            self._proc.kill()
        except Exception:
            pass
        self._proc = None
        self._out_path = None

    def reload_calibration(self) -> None:
        # Daemon auto-reloads calibration on file mtime change.
        return

    def get_jpeg(self, timeout_s: float = 2.0) -> bytes:
        if self._proc is None or self._proc.poll() is not None:
            raise RuntimeError("ArmCameraFrameClient is not running")
        if self._out_path is None:
            raise RuntimeError("ArmCameraFrameClient output path not set")

        deadline = time.time() + timeout_s
        while time.time() < deadline:
            try:
                st = self._out_path.stat()
                if st.st_mtime > self._last_mtime and st.st_size > 0:
                    data = self._out_path.read_bytes()
                    self._last_mtime = st.st_mtime
                    return data
            except FileNotFoundError:
                pass
            except Exception:
                pass
            time.sleep(0.01)

        err_tail = b""
        try:
            if self._proc.stderr is not None:
                rerr, _, _ = select.select([self._proc.stderr], [], [], 0.0)
                if rerr:
                    err_tail = self._proc.stderr.read(4096) or b""
        except Exception:
            pass
        raise RuntimeError(
            f"Timed out waiting for camera daemon to write {self._out_path}."
            + (f" Child stderr: {err_tail.decode(errors='ignore')}" if err_tail else "")
        )


if __name__ == "__main__":
    raise SystemExit(_run_child())