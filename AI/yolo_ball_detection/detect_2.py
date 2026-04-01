import argparse
from ultralytics import YOLO
import cv2

# ── CLI args ──────────────────────────────────────────────────────────────────
parser = argparse.ArgumentParser()
parser.add_argument("--jetson", action="store_true", help="Use TensorRT engine on Jetson")
args = parser.parse_args()

# ── Model loading ─────────────────────────────────────────────────────────────
if args.jetson:
    import os
    ENGINE_PATH = "./balls.engine"
    if not os.path.exists(ENGINE_PATH):
        print("No TensorRT engine found. Exporting from balls.pt ...")
        model = YOLO("./balls.pt")
        model.export(format="engine", half=True, device=0)
        print("Export complete. Restart the script with --jetson.")
        exit(0)
    model = YOLO(ENGINE_PATH)
    print("Running with TensorRT engine (FP16)")
else:
    model = YOLO("./balls.pt")
    print("Running with PyTorch model")

CONF_THRESHOLD = 0
GREEN_CLASS_INDEX = 2
CAMERA_H_FOV_DEG = 95


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
        "video/x-raw, format=(string)BGR ! appsink"
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
    cap = cv2.VideoCapture(gstreamer_pipeline(), cv2.CAP_GSTREAMER)
    print("Using GStreamer pipeline (Jetson CSI camera)")
else:
    cap = cv2.VideoCapture(0)
    print("Using default webcam")

while True:
    ret, frame = cap.read()
    if not ret:
        break

    # Run inference on the frame
    results = model(frame, verbose=False)

    best_conf = -1.0
    best_center = None
    best_box = None
    diameter = None

    for r in results:
        for box in r.boxes:
            conf = float(box.conf[0])

            # Only keep detections above threshold
            if conf > CONF_THRESHOLD:
                x1, y1, x2, y2 = map(int, box.xyxy[0])
                cls = int(box.cls[0])
                colors = ["0", "1", "green", "3", "red"]

                cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)

                cv2.putText(frame, f"{conf:.2f}", (x1, y1 - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
                # cv2.circle(frame, ((x1 + x2) / 2, (y1 + y2) / 2), 5, (0, 255, 0), -1)

                # Track ONLY the green ball with the highest confidence
                if conf > best_conf:
                    cx = int((x1 + x2) / 2)
                    cy = int((y1 + y2) / 2)
                    best_conf = conf
                    best_center = (cx, cy)
                    best_box = (x1, y1, x2, y2)
                    diameter = x2 - x1

    # If we found a green ball this frame, draw it and save the center
    # if best_center is not None and best_box is not None:
    #     x1, y1, x2, y2 = best_box
    #     cx, cy = best_center

    #     # Draw bounding box and center point
    #     cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
    #     cv2.circle(frame, (cx, cy), 5, (0, 255, 0), -1)
    #     cv2.putText(
    #         frame,
    #         f"green: {best_conf:.2f} | depth: {calculate_depth(diameter/2)} cm",
    #         (x1, y1 - 10),
    #         cv2.FONT_HERSHEY_SIMPLEX,
    #         0.6,
    #         (0, 255, 0),
    #         2,
    #     )

        # Compute required rotation and command platform
        # frame_h, frame_w = frame.shape[:2]
        # angle = compute_rotation_angle(frame_w, cx)
        # rotate_platform(angle)

        # "Save" the center (latest center always available in this file)
        # with open("green_ball_center.txt", "w") as f:
        #     f.write(f"{cx},{cy}\n")

        # print(f"Green ball center: ({cx}, {cy}) | conf={best_conf:.2f}")

    # Show the frame
    cv2.imshow("YOLO Detection", frame)

    # Press 'q' to quit
    if cv2.waitKey(1) & 0xFF == ord("q"):
        break

cap.release()
cv2.destroyAllWindows()