# import the necessary packages
from collections import deque
from imutils.video import VideoStream
import numpy as np
import argparse
import cv2
import imutils
import time

#------------- MAGIC NUMBERS -------------
camera_angle_horizontal = 62.2 #degrees - dit is al juist voor onze camera
camera_angle_vertical = 48.8 #degrees - dit is al juist voor onze camera

tan_horizontal = np.tan(np.radians(camera_angle_horizontal/2))
tan_vertical = np.tan(np.radians(camera_angle_vertical/2))

pixels_width = 600 # max 3280 pixels in onze camera
pixels_height = 450 # max 2468 pixels in onze camera

ball_radius = 5.7/2 # centimeter

k1 = ball_radius/(2*tan_horizontal)*pixels_width
k2 = ball_radius/(2*tan_vertical)*pixels_height
k = (k1+k2)/2

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

pipeline = gstreamer_pipeline(flip_method=0)

def calculate_depth(radius):
    if radius > 0:
        # k wordt nu hierboven berekend, al juist voor onze camera
        depth = k / radius
        return depth

def calculate_3D_coordinates(M_x,M_y,radius):
	"""
	(M_x, M_y)=positie middelpunt in pixels, radius in pixels
	returnt x,y,z coördinaten van de bal, waarbij assenstelsel als volgt is gedefiniëerd:
	x = horizontale as, met 0 in midden van camera, en vanuit het perspectief van de camera is meer naar rechts = hogere waarden
	y = verticale as, met 0 in midden van camera, en vanuit het perspectief van de camera is meer naar boven = hogere waarden
	z = - loodrechte afstand van bal tot camera (dus altijd negatief)
	"""
	depth = calculate_depth(radius)

	centimeters_width = 2*depth*tan_horizontal
	centimeters_height = 2*depth*tan_vertical
	x = (M_x - pixels_width/2)*centimeters_width/pixels_width
	y = -(M_y - pixels_height/2)*centimeters_height/pixels_height
	z = -depth
	return x, y, z

def white_balance(frame):
	"""Gray World white balance to compensate for colored light sources."""
	result = cv2.cvtColor(frame, cv2.COLOR_BGR2LAB)
	avg_a = np.average(result[:, :, 1])
	avg_b = np.average(result[:, :, 2])
	result[:, :, 1] = result[:, :, 1] - ((avg_a - 128) * (result[:, :, 0] / 255.0) * 1.1)
	result[:, :, 2] = result[:, :, 2] - ((avg_b - 128) * (result[:, :, 0] / 255.0) * 1.1)
	return cv2.cvtColor(result, cv2.COLOR_LAB2BGR)


def add_trackbars(): # for HSV boundaries
	def nothing(x):
		pass

	# Create a window for trackbars
	cv2.namedWindow("HSV Calibration")
	cv2.resizeWindow("HSV Calibration", 400, 300)

	# Create a window for trackbars
	cv2.namedWindow("Circle Calibration")
	cv2.resizeWindow("Circle Calibration", 400, 300)

	# Create trackbars for Lower HSV (wider S/V defaults since brightness is normalized)
	cv2.createTrackbar("H Lower", "HSV Calibration", greenLower[0], 179, nothing)
	cv2.createTrackbar("S Lower", "HSV Calibration", greenLower[1], 255, nothing)
	cv2.createTrackbar("V Lower", "HSV Calibration", greenLower[2], 255, nothing)

	# Create trackbars for Upper HSV
	cv2.createTrackbar("H Upper", "HSV Calibration", greenUpper[0], 179, nothing)
	cv2.createTrackbar("S Upper", "HSV Calibration", greenUpper[1], 255, nothing)
	cv2.createTrackbar("V Upper", "HSV Calibration", greenUpper[2], 255, nothing)

	cv2.createTrackbar("circularity cutoff", "Circle Calibration", circularity_cutoff, 100, nothing)

	cv2.createTrackbar("Hough param1", "Circle Calibration", param1, 100, nothing)
	cv2.createTrackbar("Hough param2", "Circle Calibration", param2, 50, nothing)


# construct the argument parse and parse the arguments
ap = argparse.ArgumentParser()
ap.add_argument("-v", "--video",
	help="path to the (optional) video file")
ap.add_argument("-b", "--buffer", type=int, default=64,
	help="max buffer size")
ap.add_argument("--jetson",action = "store_true", help= "Set to true if the file is run on the jetson")
args = vars(ap.parse_args())

# define the lower and upper boundaries of the "green" ball in the HSV color space
# S/V ranges are wider since lighting variation is handled by CLAHE + white balance
greenLower = (70, 50, 0)
greenUpper = (90, 255, 255)
circularity_cutoff = 50
param1 = 60
param2 = 20
pts = deque(maxlen=args["buffer"])

# if a video path was not supplied, grab the reference to the webcam
if args["jetson"]:
	vs = VideoStream(src=pipeline, usePiCamera=False).start()
elif not args.get("video", False):
	vs = VideoStream(src=0).start()
# otherwise, grab a reference to the video file
else:
	vs = cv2.VideoCapture(args["video"])

# allow the camera or video file to warm up
time.sleep(2.0)

# initialize CLAHE once (reused every frame for efficiency)
clahe = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8, 8))

add_trackbars()

# keep looping
while True:
	# grab the current frame
	frame = vs.read()
	# handle the frame from VideoCapture or VideoStream
	frame = frame[1] if args.get("video", False) else frame
	# if we are viewing a video and we did not grab a frame,
	# then we have reached the end of the video
	if frame is None:
		break
	print(frame.shape)
	# resize the frame
	frame = imutils.resize(frame, width=pixels_width)
	print(frame.shape)
	# --- Lighting normalization pipeline ---
	# Step 1: White balance to fix color casts from light sources
	frame = white_balance(frame)

	# Step 2: Blur and convert to HSV
	blurred = cv2.GaussianBlur(frame, (11, 11), 0)
	hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)

	# Step 3: Apply CLAHE to the V (brightness) channel only
	h, s, v = cv2.split(hsv)
	v = clahe.apply(v)
	hsv = cv2.merge([h, s, v])
	# --- End lighting normalization ---

	# Read trackbar positions
	h_lower = cv2.getTrackbarPos("H Lower", "HSV Calibration")
	s_lower = cv2.getTrackbarPos("S Lower", "HSV Calibration")
	v_lower = cv2.getTrackbarPos("V Lower", "HSV Calibration")

	h_upper = cv2.getTrackbarPos("H Upper", "HSV Calibration")
	s_upper = cv2.getTrackbarPos("S Upper", "HSV Calibration")
	v_upper = cv2.getTrackbarPos("V Upper", "HSV Calibration")



	greenLower = (h_lower, s_lower, v_lower)
	greenUpper = (h_upper, s_upper, v_upper)

	circularity_cutoff = cv2.getTrackbarPos("circularity cutoff", "Circle Calibration") / 100
	param1 = max(cv2.getTrackbarPos("Hough param1", "Circle Calibration"),1)
	param2 = max(cv2.getTrackbarPos("Hough param2", "Circle Calibration"),1)


	# construct a mask for the color "green", then perform
	# a series of dilations and erosions to remove any small blobs left in the mask
	if h_lower <= h_upper:
        # Normal case - no wrap-around
		mask = cv2.inRange(hsv, greenLower, greenUpper)
	else:
        # Wrap-around case: hue goes from lower_hue to 179, then 0 to upper_hue
		mask1 = cv2.inRange(hsv, (h_lower, s_lower, v_lower), 
                                  (179, s_upper, v_upper))
		mask2 = cv2.inRange(hsv, (0, s_lower, v_lower), 
                                  (h_upper, s_upper, v_upper))
		mask = cv2.bitwise_or(mask1, mask2)
	
	mask = cv2.erode(mask, None, iterations=2)
	mask = cv2.dilate(mask, None, iterations=2)


	# METHODE 1: grootste contouren vinden, daar cirkel rond fitten (GEEL)

	# find contours in the mask and initialize the current (x, y) center of the ball
	cnts = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL,
		cv2.CHAIN_APPROX_SIMPLE)
	cnts = imutils.grab_contours(cnts)

	def circularity(contour):
		area = cv2.contourArea(contour)
		perimeter = cv2.arcLength(contour, True)
		if perimeter == 0:
			return 0
		return (4 * np.pi * area) / (perimeter ** 2)


	big_cnts = [c for c in cnts if cv2.contourArea(c) > 30]
	round_cnts = [c for c in big_cnts if circularity(c) > circularity_cutoff] 

	center = None

	# only proceed if at least one contour was found
	if len(round_cnts) > 0:
		# find the largest contour in the mask, then use
		# it to compute the minimum enclosing circle and centroid
		c = max(round_cnts, key=cv2.contourArea)
		((x, y), radius) = cv2.minEnclosingCircle(c)
		M = cv2.moments(c)
		print(x, y)
		center = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))
		# only proceed if the radius meets a minimum size
		if radius > 5:
			# draw the circle and centroid on the frame,
			# then update the list of tracked points
			cv2.circle(frame, (int(x), int(y)), int(radius),
				(0, 255, 255), 2)
			cv2.circle(frame, center, 5, (0, 0, 255), -1)
			x_pos,y_pos,z_pos = calculate_3D_coordinates(x,y,radius)
			cv2.putText(frame, f"x: {int(x_pos)} cm ; y: {int(y_pos)} cm ; z: {int(z_pos)} cm", (int(x)+20, int(y)+10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)



			padding = 20
			roi = mask[max(int(y-(radius+padding)),0):min(int(y+(radius+padding)),frame.shape[0]), max(int(x-(radius+padding)),0):min(int(x+(radius+padding)),frame.shape[1])]


			circles = cv2.HoughCircles(
				roi,
				cv2.HOUGH_GRADIENT,
				dp=1.2,                # try 1.0, 1.2, 1.5, 2.0
				minDist=500,            # minimum distance between detected centers
				param1=param1,             # Canny high threshold (try 30-100)
				param2=param2,             # accumulator threshold for detection (smaller = more detections, try 15-40)
				minRadius=10,          # try >0 if your balls are large in the image
				maxRadius=0          # tune based on expected object size
			)
			if circles is not None:
				circles = np.round(circles[0, :]).astype("int")
				for (x_rel, y_rel, r) in circles:
					#cv2.circle(frame, (int(x-(radius+padding))+x_rel, int(y-(radius+padding))+y_rel), r, (255, 255, 255), 4)
					pass

	# METHODE 2: met Hough Circles (GROEN)
	# additional blurring to help edge detection
	processed_mask = cv2.medianBlur(mask, 5)

	circles = cv2.HoughCircles(
        processed_mask,
        cv2.HOUGH_GRADIENT,
        dp=1.2,                # try 1.0, 1.2, 1.5, 2.0
        minDist=500,            # minimum distance between detected centers
        param1=param1,             # Canny high threshold (try 30-100)
        param2=param2,             # accumulator threshold for detection (smaller = more detections, try 15-40)
        minRadius=10,          # try >0 if your balls are large in the image
        maxRadius=0          # tune based on expected object size
    )

	if circles is not None:
		circles = np.round(circles[0, :]).astype("int")
		for (x, y, r) in circles:
			#cv2.circle(frame, (x, y), r, (0, 255, 0), 4)
			pass




	# update the points queue
	pts.appendleft(center)

	# loop over the set of tracked points
	for i in range(1, len(pts)):
		# if either of the tracked points are None, ignore them
		if pts[i - 1] is None or pts[i] is None:
			continue
		# otherwise, compute the thickness of the line and draw the connecting lines
		thickness = int(np.sqrt(args["buffer"] / float(i + 1)) * 2.5)
		cv2.line(frame, pts[i - 1], pts[i], (0, 0, 255), thickness)

	# overlay current HSV values and active corrections on the frame
	hsv_text = f"HSV Lower: ({h_lower}, {s_lower}, {v_lower}) | Upper: ({h_upper}, {s_upper}, {v_upper})"
	cv2.putText(frame, hsv_text, (10, 30), cv2.FONT_HERSHEY_SIMPLEX,
				0.5, (255, 255, 255), 1)
	cv2.putText(frame, "Corrections: White Balance + CLAHE", (10, 55),
				cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 150), 1)

	# show the frame and mask
	cv2.imshow("Frame", frame)
	cv2.imshow("Mask", mask)

	key = cv2.waitKey(1) & 0xFF
	# if the 'q' key is pressed, stop the loop
	if key == ord("q"):
		break

# if we are not using a video file, stop the camera video stream
if not args.get("video", False):
	vs.stop()
# otherwise, release the camera
else:
	vs.release()
# close all windows
cv2.destroyAllWindows()