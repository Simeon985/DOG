import cv2
import os
import sys
import select
import termios
import tty
from datetime import datetime
import uuid
import time

def gstreamer_pipeline(
    sensor_id=0,
    capture_width=1920,
    capture_height=1080,
    display_width=960,
    display_height=540,
    framerate=30,
    flip_method=2,
):
    return (
        "nvarguscamerasrc sensor-id=%d ! "
        "video/x-raw(memory:NVMM), width=(int)%d, height=(int)%d, framerate=(fraction)%d/1 ! "
        "nvvidconv flip-method=%d ! "
        "video/x-raw, width=(int)%d, height=(int)%d, format=(string)BGRx ! "
        "videoconvert ! "
        "video/x-raw, format=(string)BGR ! "
        "appsink max-buffers=1 drop=true"  # ← changed from just "appsink"
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


def is_data():
    """Checks if there is a keypress waiting in the terminal buffer."""
    return select.select([sys.stdin], [], [], 0) == ([sys.stdin], [], [])

def capture_image_ssh():
    # Use 1080p for capture
    cap = cv2.VideoCapture(0)
    save_path = "captured_faces_laptop"
    os.makedirs(save_path, exist_ok=True)

    # Save terminal settings so we can restore them later
    old_settings = termios.tcgetattr(sys.stdin)
    
    try:
        # Switch terminal to char-by-char mode
        tty.setcbreak(sys.stdin.fileno())
        
        print("\n--- SSH REMOTE CAPTURE ---")
        print("Press 's' in this terminal to save.")
        print("Press 'q' to quit.")

        while True:
            ret, frame = cap.read()
            if not ret:
                break
            # Check if a key was pressed in the terminal
            if is_data():
                c = sys.stdin.read(1)
                if c.lower() == 's':
                    # Drain buffer to get fresh frame
                    for _ in range(5): cap.grab()
                    ret, frame = cap.retrieve()
                    
                    frame = cv2.flip(frame, -1)
                    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
                    filename = os.path.join(save_path, f"capture_{timestamp}.jpg")
                    cv2.imwrite(filename, frame)
                    print(f"\r[SUCCESS] Saved: {filename}                     ", end="")
                
                elif c.lower() == 'q':
                    print("\nExiting...")
                    break

    finally:
        # ALWAYS restore terminal settings or your terminal will act weird
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)
        cap.release()

def capture_stream():
    cap = cv2.VideoCapture(gstreamer_pipeline())
    save_path = "captured_stream"
    os.makedirs(save_path, exist_ok=True)
    while True:
        ret, frame = cap.read()
        if not ret:
            break
        im_path = os.path.join(save_path, f"ref_{time.time()}.png")
        cv2.imwrite(im_path, frame)
        #cv2.imshow("Face Recognition", frame)
        if cv2.waitKey(1) == 27:
            break
    cap.release()

if __name__ == "__main__":
    capture_stream()