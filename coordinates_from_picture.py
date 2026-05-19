import ctypes
import os
from pathlib import Path
import numpy as np
from insightface.app import FaceAnalysis
import cv2
import time
import uuid

from merged_camera import take_image

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
from processes.camera.utils import *

TEMP_MATCH_THRESHOLD = 0.4
GALLERY_MATCH_THRESHOLD = 0.2
GALLERY_UPDATE_INTERVAL = 10
TEMP_TIMEOUT = 2.0
CONFIRMATIONS_NEEDED = 5
PENDING_TIMEOUT = 10.0
CONFIRM_THRESHOLD = 0.6


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
	frame = cv2.rotate(frame, cv2.ROTATE_180)

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
	return pixels_width-Mx,pixels_height-My,radius

def get_face_middle_and_radius_from_frame_old(frame: "np.ndarray") -> tuple[float, float, float]:
    """
    Run face detection on an in-memory BGR frame and return (center_x, center_y, radius) in pixel coordinates.
    Returns (Mx, My, radius) or (None, None, None) if no face detected.
    """
    if frame is None:
        return None, None, None
    frame = cv2.rotate(frame, cv2.ROTATE_180)

    # Initialize face detection model (static/global to avoid re-initialization)
    if not hasattr(get_face_middle_and_radius_from_frame, "face_app"):
        print("Loading face detection model...")
        get_face_middle_and_radius_from_frame.face_app = FaceAnalysis(
            name='buffalo_sc',
            providers=[
                ('TensorrtExecutionProvider', {
                    'trt_engine_cache_enable': True,
                    'trt_engine_cache_path': '/home/dog/.insightface/trt_cache'
                }),
                'CUDAExecutionProvider',
                'CPUExecutionProvider'
            ]
        )
        get_face_middle_and_radius_from_frame.face_app.prepare(ctx_id=0, det_size=(640, 640))
    
    faces = get_face_middle_and_radius_from_frame.face_app.get(frame)
    
    if not faces:
        print("NO FACES DETECTED")
        return None, None, None
    
    # Use the first face detected
    face = faces[0]
    box = face.bbox.astype(int)  # [x1, y1, x2, y2]
    
    # Calculate center
    Mx = (box[0] + box[2]) / 2
    My = (box[1] + box[3]) / 2
    
    # Calculate radius (half of width)
    radius = (box[2] - box[0]) / 2
    
    print(f"Face detected - center: ({Mx:.1f}, {My:.1f}), radius: {radius:.1f}")
    return pixels_width-Mx,pixels_height-My, radius


def get_face_middle_and_radius_from_frame(frame: "np.ndarray") -> tuple[float, float, float, str]:
    """
    Run face detection on an in-memory BGR frame and return (center_x, center_y, radius, name) in pixel coordinates.
    Returns (Mx, My, radius, name) or (None, None, None, None) if no face detected.
    """
    if frame is None:
        return None, None, None, None
    frame = cv2.rotate(frame, cv2.ROTATE_180)

    # ── Initialize model + galleries once ─────────────────────────────────────
    if not hasattr(get_face_middle_and_radius_from_frame, "face_app"):
        print("Loading face detection model...")
        get_face_middle_and_radius_from_frame.face_app = FaceAnalysis(
            name='buffalo_sc',
            providers=[
                ('TensorrtExecutionProvider', {
                    'trt_engine_cache_enable': True,
                    'trt_engine_cache_path': '/home/dog/.insightface/trt_cache'
                }),
                'CUDAExecutionProvider',
                'CPUExecutionProvider'
            ]
        )
        get_face_middle_and_radius_from_frame.face_app.prepare(ctx_id=0, det_size=(640, 640))

        # Load known faces from images
        known_faces = {}
        npy_faces = set()
        images_dir = 'AI/images'
        embeddings_dir = 'AI/images/embeddings'

        print("Loading known faces from images...")
        for filename in ['robin.jpg', 'thomas.jpg', 'jorien.jpg', 'Wannes.jpg', 'kobe.jpeg','august.png','thomasVA.png','yente.png','yente2.png', 'simeon.jpg']:
            image_path = os.path.join(images_dir, filename)
            if os.path.exists(image_path):
                img = cv2.imread(image_path)
                if img is not None:
                    faces = get_face_middle_and_radius_from_frame.face_app.get(img)
                    if len(faces) > 0:
                        name = os.path.splitext(filename)[0]
                        known_faces[name] = [faces[0].normed_embedding]
                        print(f"Loaded face: {name}")
                    else:
                        print(f"No face detected in {filename}")
                else:
                    print(f"Could not read {filename}")
            else:
                print(f"File not found: {image_path}")

        os.makedirs(embeddings_dir, exist_ok=True)
        for filename in os.listdir(embeddings_dir):
            if filename.endswith('.npz'):
                name = os.path.splitext(filename)[0]
                if name not in known_faces:
                    known_faces[name] = load_gallery(os.path.join(embeddings_dir, filename))
                    npy_faces.add(name)
                    print(f"Loaded gallery: {name} ({len(known_faces[name])} embeddings)")

        print(f"Loaded {len(known_faces)} known faces")

        # Attach persistent state to the function
        get_face_middle_and_radius_from_frame.known_faces  = known_faces
        get_face_middle_and_radius_from_frame.npy_faces    = npy_faces
        get_face_middle_and_radius_from_frame.temp_faces   = {}
        get_face_middle_and_radius_from_frame.pending_faces = {}

    # Convenience aliases
    app           = get_face_middle_and_radius_from_frame.face_app
    known_faces   = get_face_middle_and_radius_from_frame.known_faces
    npy_faces     = get_face_middle_and_radius_from_frame.npy_faces
    temp_faces    = get_face_middle_and_radius_from_frame.temp_faces
    pending_faces = get_face_middle_and_radius_from_frame.pending_faces

    # ── Expire stale tracks ────────────────────────────────────────────────────
    now = time.time()
    temp_faces    = {k: v for k, v in temp_faces.items()    if now - v['last_seen'] < TEMP_TIMEOUT}
    pending_faces = {k: v for k, v in pending_faces.items() if now - v['last_seen'] < PENDING_TIMEOUT}
    get_face_middle_and_radius_from_frame.temp_faces    = temp_faces
    get_face_middle_and_radius_from_frame.pending_faces = pending_faces

    faces = app.get(frame)

    if not faces:
        print("NO FACES DETECTED")
        return None, None, None, None

    # Use the largest face (by box area) as the primary target
    face = max(faces, key=lambda f: (f.bbox[2] - f.bbox[0]) * (f.bbox[3] - f.bbox[1]))
    emb  = face.normed_embedding
    box  = face.bbox.astype(int)

    # ── Step 1: match against temp embeddings ─────────────────────────────────
    best_temp     = None
    best_temp_sim = 0.0
    for track_id, data in temp_faces.items():
        sim = float(np.dot(emb, data['embedding']))
        if sim > best_temp_sim:
            best_temp_sim = sim
            best_temp     = track_id

    recognized_name = None   # will stay None if unknown

    if False:#best_temp and best_temp_sim >= TEMP_MATCH_THRESHOLD:
        temp_faces[best_temp]['embedding']    = emb
        temp_faces[best_temp]['frame_count'] += 1
        temp_faces[best_temp]['last_seen']    = now
        name = temp_faces[best_temp]['name']

        if (name and name in known_faces
                and temp_faces[best_temp]['frame_count'] % GALLERY_UPDATE_INTERVAL == 0):
            updated, was_updated = try_add_to_gallery(emb, known_faces[name])
            if was_updated:
                known_faces[name] = updated
                if name in npy_faces:
                    save_gallery(name, updated)
                print(f"{name}: gallery updated ({len(updated)} embeddings)")

        recognized_name = name  # may be None if this temp track is still pending

    # ── Step 2: no temp match — check gallery ─────────────────────────────────
    else:
        best_gallery_name = None
        best_gallery_sim  = 0.0
        best_gallery_emb  = None
        for name, gallery in known_faces.items():
            sim, idx = match_gallery(emb, gallery)
            if sim > best_gallery_sim:
                best_gallery_sim  = sim
                best_gallery_name = name
                best_gallery_emb  = gallery[idx]

        if best_gallery_name and best_gallery_sim >= GALLERY_MATCH_THRESHOLD:
            track_id = uuid.uuid4().hex[:8]
            temp_faces[track_id] = {
                'embedding':   best_gallery_emb,
                'name':        best_gallery_name,
                'frame_count': 1,
                'last_seen':   now,
            }
            print(f"Gallery match: {best_gallery_name} ({best_gallery_sim:.2f}) → temp {track_id}")
            recognized_name = best_gallery_name

        # ── Step 3: no gallery match — check/create pending ───────────────────
        else:
            best_pending     = None
            best_pending_sim = 0.0
            for temp_id, data in pending_faces.items():
                sim, _ = match_gallery(emb, data['gallery'])
                if sim > best_pending_sim:
                    best_pending_sim = sim
                    best_pending     = temp_id

            if best_pending and best_pending_sim >= CONFIRM_THRESHOLD:
                pending_faces[best_pending]['confirmations'] += 1
                pending_faces[best_pending]['last_seen']      = now
                pending_faces[best_pending]['gallery'], _     = try_add_to_gallery(
                    emb, pending_faces[best_pending]['gallery'], pending=True
                )
                confs = pending_faces[best_pending]['confirmations']
                print(f"Pending {best_pending}: {confs}/{CONFIRMATIONS_NEEDED} confirmations")

                if confs >= CONFIRMATIONS_NEEDED:
                    new_name = f"unknown_{uuid.uuid4().hex[:8]}"
                    known_faces[new_name] = pending_faces.pop(best_pending)['gallery']
                    npy_faces.add(new_name)
                    save_gallery(new_name, known_faces[new_name])
                    print(f"Face confirmed and saved as: {new_name}")
                    recognized_name = new_name
            else:
                temp_id = uuid.uuid4().hex[:8]
                pending_faces[temp_id] = {
                    'gallery':       [emb],
                    'confirmations': 1,
                    'last_seen':     now,
                }
                print(f"New pending face: {temp_id}")

    # ── Geometry ───────────────────────────────────────────────────────────────
    Mx     = (box[0] + box[2]) / 2
    My     = (box[1] + box[3]) / 2
    radius = (box[2] - box[0]) / 2

    label = recognized_name if recognized_name else "Unknown"
    print(f"Face detected - center: ({Mx:.1f}, {My:.1f}), radius: {radius:.1f}, name: {label}")

    return pixels_width - Mx, pixels_height - My, radius, recognized_name


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

def get_face_middle_and_radius_from_picture():
	image_path = Path(__file__).with_name("image.png")
	take_image(image_path)
	img = cv2.imread(str(image_path))
	if img is None:
		raise RuntimeError(f"Failed to read image: {image_path}")
	return get_face_middle_and_radius_from_frame(img)


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