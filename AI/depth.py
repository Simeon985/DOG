import cv2
import numpy as np

cap = cv2.VideoCapture(0)

# Good site for this: https://pseudopencv.site/utilities/hsvcolormask/
greenLower = (40, 0, 33)
greenUpper = (90, 180, 255)

def calculate_depth(radius):
    if radius > 0:
        # Als r = 11, dan is de afstand 156 cm
        # Depth moet kleiner worden als de radius groter is (omgekeerd evenredig)
        # Bijvoorbeeld: depth = k / radius, waarbij k een kalibratieconstante is
        # Gegeven: als r = 11, dan is de afstand 156 cm -> k = 11 * 156 = 1716
        depth = 1716 / radius
        return depth

while True:
    ret, frame = cap.read()
    if not ret:
        break


    blurred = cv2.GaussianBlur(frame, (11, 11), 0)
    hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)

    mask = cv2.inRange(hsv, greenLower, greenUpper)
    mask = cv2.erode(mask, None, iterations=2)
    mask = cv2.dilate(mask, None, iterations=2)

    # Param1 = sensitivity, Param2 = threshold

    # Apply additional blurring to help edge detection
    processed_mask = cv2.medianBlur(mask, 5)

    circles = cv2.HoughCircles(
        processed_mask,
        cv2.HOUGH_GRADIENT,
        dp=1.2,                # try 1.0, 1.2, 1.5, 2.0
        minDist=500,            # minimum distance between detected centers
        param1=60,             # Canny high threshold (try 30-100)
        param2=30,             # accumulator threshold for detection (smaller = more detections, try 15-40)
        minRadius=10,          # try >0 if your balls are large in the image
        maxRadius=0          # tune based on expected object size
    )

    if circles is not None:
        circles = np.round(circles[0, :]).astype("int")
        for (x, y, r) in circles:
            cv2.circle(frame, (x, y), r, (0, 255, 0), 4)
            cv2.putText(frame, f"{calculate_depth(r)} cm", (x, y), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)

    cv2.imshow("Frame", frame)

    if cv2.waitKey(1) == 27:
        break

cap.release()
cv2.destroyAllWindows()