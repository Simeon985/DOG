import argparse
import threading
from ultralytics import YOLO
import cv2

# ── CLI args ──────────────────────────────────────────────────────────────────
parser = argparse.ArgumentParser()
parser.add_argument("--jetson", action="store_true", help="Use TensorRT engine on Jetson")
args = parser.parse_args()

# ── Model loading ─────────────────────────────────────────────────────────────
if args.jetson:
    import os
    import torch
    ENGINE_PATH = "./balls.engine"
    if not os.path.exists(ENGINE_PATH):
        print("No TensorRT engine found. Exporting from balls.pt ...")
        model = YOLO("./balls.pt")
        device = 0 if torch.cuda.is_available() else "cpu"
        model.export(format="engine", half=False, device=device, workspace=2, batch=1)
        print("Export complete. Restart the script with --jetson.")
        exit(0)
    model = YOLO(ENGINE_PATH, task = "detect")
    print("Running with TensorRT engine (FP32)")
else:
    model = YOLO("./balls.pt")
    print("Running with PyTorch model")

CONF_THRESHOLD = 0
CAMERA_H_FOV_DEG = 95

# ── Camera stream class (threaded) ────────────────────────────────────────────
class CameraStream:
    def __init__(self, src, backend=None):
        self.cap = cv2.VideoCapture(src, backend) if backend else cv2.VideoCapture(src)
        self.ret, self.frame = self.cap.read()
        self.lock = threading.Lock()
        self.running = True
        self.thread = threading.Thread(target=self._update, daemon=True)
        self.thread.start()

    def _update(self):
        while self.running:
            ret, frame = self.cap.read()
            with self.lock:
                self.ret, self.frame = ret, frame

    def read(self):
        with self.lock:
            return self.ret, self.frame.copy() if self.frame is not None else None

    def release(self):
        self.running = False
        self.thread.join()
        self.cap.release()


def gstreamer_pipeline(
    sensor_id=0,
    capture_width=1920,
    capture_height=1080,
    display_width=960,
    display_height=540,
    framerate=30,
    flip_method=0,
):
    return (
        "nvarguscamerasrc sensor-id=%d ! "
        "video/x-raw(memory:NVMM), width=(int)%d, height=(int)%d, framerate=(fraction)%d/1 ! "
        "nvvidconv flip-method=%d ! "
        "video/x-raw, width=(int)%d, height=(int)%d, format=(string)BGRx ! "
        "videoconvert ! "
        "video/x-raw, format=(string)BGR ! appsink drop=1 max-buffers=1"
        % (
            sensor_id,
            capture_width,
            capture_height,
            framerate,
            flip_method,
            display_width,
            display_height,
        )
    )


def calculate_depth(radius):
    if radius > 0:
        depth = 1716 / radius
        return round(depth, 1)


def compute_rotation_angle(frame_width, target_x):
    center_x = frame_width / 2.0
    pixel_offset = target_x - center_x
    norm_offset = pixel_offset / (frame_width / 2.0)
    angle = norm_offset * (CAMERA_H_FOV_DEG / 2.0)
    return angle


def rotate_platform(angle):
    print(f"rotate_platform({angle:.2f} deg)")


# ── Camera source ─────────────────────────────────────────────────────────────
if args.jetson:
    cap = CameraStream(gstreamer_pipeline(), cv2.CAP_GSTREAMER)
    print("Using GStreamer pipeline (Jetson CSI camera)")
else:
    cap = CameraStream(0)
    print("Using default webcam")

# ── Inference loop ────────────────────────────────────────────────────────────
while True:
    ret, frame = cap.read()
    if not ret or frame is None:
        continue
    results = model(frame, verbose=False)

    best_conf = -1.0
    best_center = None
    best_box = None
    diameter = None

    for r in results:
        for box in r.boxes:
            conf = float(box.conf[0])
            if conf > CONF_THRESHOLD:
                x1, y1, x2, y2 = map(int, box.xyxy[0])
                cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
                cv2.putText(frame, f"{conf:.2f}", (x1, y1 - 10),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)

                if conf > best_conf:
                    cx = int((x1 + x2) / 2)
                    cy = int((y1 + y2) / 2)
                    best_conf = conf
                    best_center = (cx, cy)
                    best_box = (x1, y1, x2, y2)
                    diameter = x2 - x1

    cv2.imshow("YOLO Detection", frame)

    if cv2.waitKey(1) & 0xFF == ord("q"):
        break

cap.release()
cv2.destroyAllWindows()
