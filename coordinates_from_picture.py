import ctypes
import os
from pathlib import Path
import numpy as np

from arm_camera import take_image

_conda_prefix = os.environ.get("CONDA_PREFIX")
if _conda_prefix:
    _libgio = os.path.join(_conda_prefix, "lib", "libgio-2.0.so.0")
    if os.path.exists(_libgio):
        try:
            ctypes.CDLL(_libgio, mode=ctypes.RTLD_GLOBAL)
        except OSError:
            pass

import torch
from ultralytics import YOLO
import imutils

YOLO_DEVICE = 0 if torch.cuda.is_available() else "cpu"
import cv2

camera_angle_horizontal = 52 #degrees - dit is al juist voor onze camera   - voor webcam Thomas: 66
camera_angle_vertical = 28.8 #degrees - dit is al juist voor onze camera    - voor webcam Thomas: 40

tan_horizontal = np.tan(np.radians(camera_angle_horizontal/2))
tan_vertical = np.tan(np.radians(camera_angle_vertical/2))

pixels_width = 640#3280 # max 3280 pixels in onze camera
pixels_height = 360#2468 # max 2468 pixels in onze camera

ball_radius = 6/2 # centimeter

k1 = ball_radius/(2*tan_horizontal)*pixels_width
k2 = ball_radius/(2*tan_vertical)*pixels_height
k = (k1+k2)/2

model = YOLO(str(Path(__file__).parent / "models" / "balls_ourdata_augmented.pt"))

def calculate_depth(radius):
    if radius > 0:
        # k wordt nu hierboven berekend, al juist voor onze camera
        depth = k / radius
        return depth
def from_depth_to_radius(depth):
	if depth > 0:
		radius = k / depth
		return radius
	else:
		return 1

def calculate_3D_coordinates(M_x,M_y,radius):
	"""
	(M_x, M_y)=positie middelpunt in pixels, radius in pixels
	returnt x,y,z coördinaten van de bal, waarbij assenstelsel als volgt is gedefiniëerd:
	x = horizontale as, met 0 in midden van camera, en vanuit het perspectief van de camera is meer naar rechts = hogere waarden
	y = verticale as, met 0 in midden van camera, en vanuit het perspectief van de camera is meer naar boven = hogere waarden
	z = - loodrechte afstand van bal tot camera (dus altijd negatief)
	"""
	depth = calculate_depth(radius)
	z = -depth

	centimeters_width = 2*depth*tan_horizontal
	centimeters_height = 2*depth*tan_vertical

	x = (M_x - pixels_width/2)*centimeters_width/pixels_width
	y = -(M_y - pixels_height/2)*centimeters_height/pixels_height

	return x, y, z


def get_M_and_radius_from_frame(frame: "np.ndarray") -> tuple[float, float, float]:
	"""
	Run detection on an in-memory BGR frame and return (x_cm, y_cm, z_cm) in camera coordinates.
	"""
	if frame is None:
		return None,None,None

	# Ultralytics can take numpy arrays directly. Keep it in-memory for realtime performance.

	results = model.predict(frame, verbose=False, device=YOLO_DEVICE)

	

	if not results:
		return None,None,None
	result = results[0]
	print(result.boxes)
	if result.boxes is None or len(result.boxes) == 0:
		print("NO BOXES")
		return None,None,None

	print("BOXES:")
	print(result.boxes)
	coordinates = result.boxes.xyxy[0].tolist()
	print("DETECTED!!")

	Mx = (coordinates[0] + coordinates[2]) / 2
	My = (coordinates[1] + coordinates[3]) / 2
	radius = (coordinates[2] - coordinates[0]) / 2
	print("Mx, My, radius")
	print(Mx, My, radius)
	return Mx,My,radius

def get_coordinates_from_frame(frame: "np.ndarray") -> tuple[float, float, float]:
	Mx,My,radius = get_M_and_radius_from_frame(frame)
	if Mx == None:
		return None, None, None
	x, y, z = calculate_3D_coordinates(Mx, My, radius)
	print("x, y, z")
	print(x, y, z)
	return -x,-y, z


def _capture_frame(timeout_sec: float = 3.0):
	from camera import apply_color_correction, initialize_camera, read_bgr_frame, get_video_capture

	cap = get_video_capture()
	print(f"[coords] _capture_frame: cap exists={cap is not None}, opened={cap.isOpened() if cap is not None else 'N/A'}")
	if cap is None or not cap.isOpened():
		initialize_camera(model_path=str(Path(__file__).parent / "models" / "balls_ourdata_augmented.pt"))
	ret, frame = read_bgr_frame(timeout_sec=timeout_sec)
	print(f"[coords] _capture_frame: read_bgr_frame returned ret={ret}, frame_ok={frame is not None}")
	if not ret or frame is None:
		print("[coords] _capture_frame: failed to get frame")
		return None

	frame = apply_color_correction(frame)
	image_path = Path(__file__).with_name("image.png")
	cv2.imwrite(str(image_path), frame)
	return frame


def get_coordinates_from_picture():
	image_path = Path(__file__).with_name("image.png")
	take_image(image_path)
	img = cv2.imread(str(image_path))
	if img is None:
		raise RuntimeError(f"Failed to read image: {image_path}")
	return get_coordinates_from_frame(img)

def get_M_and_radius_from_picture():
	image_path = Path(__file__).with_name("image.png")
	take_image(image_path)
	img = cv2.imread(str(image_path))
	if img is None:
		raise RuntimeError(f"Failed to read image: {image_path}")
	return get_M_and_radius_from_frame(img)


def get_coordinates_from_picture_2():
	# OpenCV HSV ranges: H=0..179, S=0..255, V=0..255
	# Use a tighter green band and require some saturation/value to avoid all-black/white masks
	greenLower = (35, 60, 40)
	greenUpper = (85, 255, 255)
	circularity_cutoff = 0.3
	img = _capture_frame()
	if img is None:
		raise RuntimeError("Failed to capture frame")
	print("size")
	print(img.shape)
	blurred = cv2.GaussianBlur(img, (11, 11), 0)
	hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)
	# Diagnostics: quickly inspect channel ranges to tune thresholds
	h, s, v = cv2.split(hsv)
	print(f"H range: {int(h.min())}-{int(h.max())}, S range: {int(s.min())}-{int(s.max())}, V range: {int(v.min())}-{int(v.max())}")
	mask = cv2.inRange(hsv, np.array(greenLower, dtype=np.uint8), np.array(greenUpper, dtype=np.uint8))
	cnts = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)
	cnts = imutils.grab_contours(cnts)

	def circularity(contour):
		area = cv2.contourArea(contour)
		perimeter = cv2.arcLength(contour, True)
		if perimeter == 0:
			return 0
		return (4 * np.pi * area) / (perimeter ** 2)

	big_cnts = [c for c in cnts if cv2.contourArea(c) > 30]
	round_cnts = [c for c in big_cnts if circularity(c) > circularity_cutoff]

	if len(round_cnts) > 0:
		c = max(round_cnts, key=cv2.contourArea)
		((M_x, M_y), radius) = cv2.minEnclosingCircle(c)
		if radius > 20:
			x_pos,y_pos,z_pos = calculate_3D_coordinates(M_x, M_y,radius)
			print(f"x: {x_pos:.1f} cm ; y: {y_pos:.1f} cm ; z: {z_pos:.1f} cm")
			return x_pos, y_pos, z_pos
	raise RuntimeError("No suitable round contour found")