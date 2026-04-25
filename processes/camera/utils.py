import time
from multiprocessing.sharedctypes import SynchronizedArray
import cv2
import numpy as np
from insightface.app import FaceAnalysis
import os
import uuid
import time
from processes.camera.coordinates_from_picture import *


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

def save_gallery(name: str, gallery: list, save_dir: str = 'AI/images/embeddings') -> None:
    """Save a person's embedding gallery as a single .npz file."""
    os.makedirs(save_dir, exist_ok=True)
    np.savez(os.path.join(save_dir, f"{name}.npz"), *gallery)

def load_gallery(path: str) -> list:
    """Load a person's embedding gallery from a .npz file."""
    data = np.load(path)
    return [data[key] for key in data]


def match_gallery(emb: np.ndarray, gallery: list) -> float:
    """Return the best similarity score across all embeddings in a gallery."""
    return max(float(np.dot(emb, known_emb)) for known_emb in gallery)


def update_gallery(emb: np.ndarray, gallery: list, similarity: float) -> list:
    """
    Add embedding to gallery only if:
    - The match was confident (high enough to be a real match)
    - The embedding is diverse enough (not too similar to any existing one)
    If gallery is full, replace the oldest entry.
    """
    GALLERY_MAX_SIZE = 10       # max embeddings stored per person
    GALLERY_ADD_THRESHOLD = 0.65 # only add to gallery if match is confident
    GALLERY_MIN_DIVERSITY = 0.10 # only add if embedding is different enough from all existing ones
    if similarity < GALLERY_ADD_THRESHOLD:
        return gallery  # not confident enough to update

    # Check diversity — skip if too similar to any existing embedding
    for existing in gallery:
        if float(np.dot(emb, existing)) > (1.0 - GALLERY_MIN_DIVERSITY):
            return gallery  # redundant embedding, skip

    if len(gallery) >= GALLERY_MAX_SIZE:
        gallery.pop(0)  # remove oldest

    gallery.append(emb)
    return gallery