import ctypes
import os
from pathlib import Path
import numpy as np
from insightface.app import FaceAnalysis

_conda_prefix = os.environ.get("CONDA_PREFIX")
if _conda_prefix:
	_libgio = os.path.join(_conda_prefix, "lib", "libgio-2.0.so.0")
	if os.path.exists(_libgio):
		try:
			ctypes.CDLL(_libgio, mode=ctypes.RTLD_GLOBAL)
		except OSError:
			pass

def initialize_coordinate_detection_faces():
	"""Initialize the magic numbers for coordinate extraction"""
	camera_angle_horizontal = 62.2 #degrees - dit is al juist voor onze camera   - voor webcam Thomas: 66
	camera_angle_vertical = 48.8 #degrees - dit is al juist voor onze camera    - voor webcam Thomas: 40

	tan_horizontal = np.tan(np.radians(camera_angle_horizontal/2))
	tan_vertical = np.tan(np.radians(camera_angle_vertical/2))

	pixels_width = 1280#3280 # max 3280 pixels in onze camera
	pixels_height = 720#2468 # max 2468 pixels in onze camera

	face_width = 12.5 # centimeter
	face_height = 18.5 #centimeter

	k_width = face_width/(2*tan_horizontal)*pixels_width
	k_height = face_height/(2*tan_vertical)*pixels_height
	return k_width, k_height, tan_horizontal, tan_vertical, pixels_width, pixels_height

def calculate_depth_faces(bbox, cam_info):
	# k wordt nu hierboven berekend, al juist voor onze camera
	k_width = cam_info[0]
	k_height = cam_info[1]
	width_px  = (bbox[2] - bbox[0])
	height_px = (bbox[3] - bbox[1])

	depth_from_width  = k_width  / width_px
	depth_from_height = k_height / height_px

	depth = (depth_from_width + depth_from_height) / 2
	return depth


def calculate_3D_coordinates_faces(face, cam_info):
	"""
	(M_x, M_y)=positie middelpunt in pixels, radius in pixels
	returnt x,y,z coördinaten van de bal, waarbij assenstelsel als volgt is gedefiniëerd:
	x = horizontale as, met 0 in midden van camera, en vanuit het perspectief van de camera is meer naar rechts = hogere waarden
	y = verticale as, met 0 in midden van camera, en vanuit het perspectief van de camera is meer naar boven = hogere waarden
	z = - loodrechte afstand van bal tot camera (dus altijd negatief)
	"""
	bbox = face.bbox.astype(int) # [x1, y1, x2, y2]
	M_x = (bbox[0] + bbox[2])/2
	M_y = (bbox[1] + bbox[3])/2
	depth = calculate_depth_faces(bbox, cam_info)
	z = -depth

	tan_horizontal = cam_info[2]
	tan_vertical = cam_info[3]
	pixels_width = cam_info[4]
	pixels_height = cam_info[5]

	centimeters_width = 2*depth*tan_horizontal
	centimeters_height = 2*depth*tan_vertical

	x = (M_x - pixels_width/2)*centimeters_width/pixels_width
	y = -(M_y - pixels_height/2)*centimeters_height/pixels_height
	return x, y, z