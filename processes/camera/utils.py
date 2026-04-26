import time
from multiprocessing.sharedctypes import SynchronizedArray
import cv2
import numpy as np
from insightface.app import FaceAnalysis
import os
import uuid
import json
from processes.camera.coordinates_from_picture import *

CALIBRATION_FILE = "processes/camera/color_calibration.json"


def save_embedding(name: str, embedding: np.ndarray, save_dir: str = 'AI/images/embeddings') -> None:
    os.makedirs(save_dir, exist_ok=True)
    np.save(os.path.join(save_dir, f"{name}.npy"), embedding)

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

def save_gallery(name: str, gallery: list, save_dir: str = 'AI/images/embeddings') -> None:
    os.makedirs(save_dir, exist_ok=True)
    np.savez(os.path.join(save_dir, f"{name}.npz"), *gallery)


def load_gallery(path: str) -> list:
    data = np.load(path)
    return [data[key] for key in data]


def match_gallery(emb: np.ndarray, gallery: list) -> tuple[float, int]:
    """Return (best_similarity, best_index) across all embeddings in a gallery."""
    sims = [float(np.dot(emb, g)) for g in gallery]
    best_idx = int(np.argmax(sims))
    return sims[best_idx], best_idx


def try_add_to_gallery(emb: np.ndarray, gallery: list, pending: bool = False) -> tuple[list, bool]:
    """
    Try to add emb to gallery.
    - If diverse enough (max sim < GALLERY_DIVERSITY_THRESHOLD): add or replace most similar if full.
    - Returns (updated_gallery, was_updated).
    """
    GALLERY_DIVERSITY_THRESHOLD_NO_PENDING = 0.6  # temp embedding must be below this to be added to gallery
    GALLERY_DIVERSITY_THRESHOLD_PENDING = 0.85
    GALLERY_MAX_SIZE = 20
    GALLERY_DIVERSITY_THRESHOLD = GALLERY_DIVERSITY_THRESHOLD_NO_PENDING if not pending else GALLERY_DIVERSITY_THRESHOLD_PENDING

    best_sim, best_idx = match_gallery(emb, gallery)

    if best_sim >= GALLERY_DIVERSITY_THRESHOLD:
        return gallery, False  # too similar to existing — redundant

    if len(gallery) >= GALLERY_MAX_SIZE:
        gallery[best_idx] = emb  # replace most similar
    else:
        gallery.append(emb)

    return gallery, True

def load_matrix():
    with open(CALIBRATION_FILE, "r") as f:
        return np.array(json.load(f), dtype=np.float32)

def apply_matrix(image, M):
    img = image.astype(np.float32) / 255.0
    img = np.dot(img, M)
    img = np.clip(img, 0, 1)
    return (img * 255).astype(np.uint8)

def sharpness_score(frame, bbox):
    x1, y1, x2, y2 = bbox.astype(int)
    # Clamp to frame boundaries
    x1 = max(0, x1)
    y1 = max(0, y1)
    x2 = min(frame.shape[1], x2)
    y2 = min(frame.shape[0], y2)
    
    # Check crop is valid after clamping
    if x2 <= x1 or y2 <= y1:
        return 0.0
    
    face_crop = frame[y1:y2, x1:x2]
    gray = cv2.cvtColor(face_crop, cv2.COLOR_BGR2GRAY)
    score = cv2.Laplacian(gray, cv2.CV_64F).var()
    return score

clahe = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8, 8))

def apply_clahe(frame):
    lab = cv2.cvtColor(frame, cv2.COLOR_BGR2LAB)
    lab[:, :, 0] = clahe.apply(lab[:, :, 0])
    return cv2.cvtColor(lab, cv2.COLOR_LAB2BGR)