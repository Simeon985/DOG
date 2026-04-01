from ultralytics import YOLO
import cv2

# Load your custom model
model = YOLO("./balls.pt")

# Open webcam (0 = default camera)
cap = cv2.VideoCapture(0)

CONF_THRESHOLD = 0
GREEN_CLASS_INDEX = 2  # index in `colors` that corresponds to "green"
CAMERA_H_FOV_DEG = 95  # adjust to your camera's horizontal field of view


def calculate_depth(radius):
    if radius > 0:
        # Als r = 11, dan is de afstand 156 cm
        # Depth moet kleiner worden als de radius groter is (omgekeerd evenredig)
        # Bijvoorbeeld: depth = k / radius, waarbij k een kalibratieconstante is
        # Gegeven: als r = 11, dan is de afstand 156 cm -> k = 11 * 156 = 1716
        depth = 1716 / radius
        return round(depth, 1)


def compute_rotation_angle(frame_width, target_x):
    """
    Compute platform rotation angle so that the camera center points toward target_x.
    Positive angle => rotate right, negative => rotate left (convention can be adapted).
    """
    center_x = frame_width / 2.0
    pixel_offset = target_x - center_x  # >0 if ball is to the right of center
    norm_offset = pixel_offset / (frame_width / 2.0)  # -1 .. 1
    angle = norm_offset * (CAMERA_H_FOV_DEG / 2.0)
    return angle


def rotate_platform(angle):
    """
    Stub for platform rotation. Replace with actual motor control implementation.
    """
    print(f"rotate_platform({angle:.2f} deg)")


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