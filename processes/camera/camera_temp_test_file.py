from utils import *
import uuid
from skimage.exposure import match_histograms
import time


TEMP_MATCH_THRESHOLD = 0.4
GALLERY_MATCH_THRESHOLD = 0.4
GALLERY_UPDATE_INTERVAL = 10
TEMP_TIMEOUT = 2.0
CONFIRMATIONS_NEEDED = 5
PENDING_TIMEOUT = 10.0
CONFIRM_THRESHOLD = 0.6
BBOX_MARGIN = 0.3  # 30% margin around previous bounding box to compensate for movement


def correct_face_region(frame: np.ndarray, bbox: np.ndarray, references: list) -> np.ndarray:
    """
    Apply color correction to a face region in-place.
    bbox should already include margin and be clamped to frame bounds.
    """
    x1, y1, x2, y2 = bbox
    face = frame[y1:y2, x1:x2].copy()

    if face.size == 0:
        return frame

    # Step 1: Histogram matching against closest reference
    if references:
        best_ref = None
        best_score = float('inf')
        face_hist = cv2.calcHist([face], [0, 1, 2], None, [8, 8, 8], [0, 256] * 3)
        for ref in references:
            ref_hist = cv2.calcHist([ref], [0, 1, 2], None, [8, 8, 8], [0, 256] * 3)
            score = cv2.compareHist(face_hist, ref_hist, cv2.HISTCMP_BHATTACHARYYA)
            if score < best_score:
                best_score = score
                best_ref = ref
        face = match_histograms(face, best_ref, channel_axis=-1).astype(np.uint8)

    # # Step 2: Gray world to fix any residual per-frame color cast
    # mean = face.mean(axis=(0, 1))
    # scale = mean.mean() / (mean + 1e-6)
    # face = np.clip(face * scale, 0, 255).astype(np.uint8)

    # # Step 3: CLAHE for local contrast
    # lab = cv2.cvtColor(face, cv2.COLOR_BGR2LAB)
    # lab[:, :, 0] = clahe.apply(lab[:, :, 0])
    # face = cv2.cvtColor(lab, cv2.COLOR_LAB2BGR)

    # Write corrected region back into frame
    frame[y1:y2, x1:x2] = face
    return frame


def expand_bbox(bbox: np.ndarray, margin: float, frame_shape: tuple) -> tuple:
    """
    Expand a bounding box by a margin fraction and clamp to frame bounds.
    Returns (x1, y1, x2, y2) as ints.
    """
    x1, y1, x2, y2 = bbox.astype(float)
    w, h = x2 - x1, y2 - y1
    x1 = max(0, int(x1 - w * margin))
    y1 = max(0, int(y1 - h * margin))
    x2 = min(frame_shape[1], int(x2 + w * margin))
    y2 = min(frame_shape[0], int(y2 + h * margin))
    return x1, y1, x2, y2


def camera_process() -> None:
    """Process running the camera / vision logic."""
    print("Loading model...")
    app = FaceAnalysis(
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
    app.prepare(ctx_id=0, det_size=(640, 640))

    known_faces = {}
    npy_faces = set()
    images_dir = 'AI/images'
    embeddings_dir = 'AI/images/embeddings'
    references_dir = 'AI/images/references'

    # Load reference face crops for histogram matching
    # These should be face crops (not full frames) taken under ideal lighting
    references = []
    os.makedirs(references_dir, exist_ok=True)
    for filename in os.listdir(references_dir):
        if filename.endswith(('.jpg', '.png')):
            ref = cv2.imread(os.path.join(references_dir, filename))
            if ref is not None:
                references.append(ref)
                print(f"Loaded reference: {filename}")
    print(f"Loaded {len(references)} reference images")

    print("Loading known faces from images...")
    for filename in ['robin.jpg', 'thomas.jpg', 'jorien.jpg', 'Wannes.jpg']:
        image_path = os.path.join(images_dir, filename)
        if os.path.exists(image_path):
            img = cv2.imread(image_path)
            if img is not None:
                faces = app.get(img)
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
    print("Starting webcam...")

    cap = cv2.VideoCapture(0)
    cam_info_faces = initialize_coordinate_detection_faces()

    M = load_matrix()

    #hared_array[10] = 1.0
    timestamp = time.time()

    temp_faces = {}
    pending_faces = {}

    # Stores bounding boxes from the previous frame
    # { track_id -> expanded_bbox (x1, y1, x2, y2) }
    prev_bboxes = []

    while True:
        ret, frame = cap.read()

        if not ret:
            break

        # ── Color correct previous frame's face regions ────────────────────
        # We use previous bounding boxes because we don't know where faces
        # are in the current frame yet — InsightFace hasn't run on it.
        # The margin compensates for movement between frames.
        for bbox in prev_bboxes:
            frame = correct_face_region(frame, bbox, references)

        now = time.time()

        temp_faces    = {k: v for k, v in temp_faces.items()    if now - v['last_seen'] < TEMP_TIMEOUT}
        pending_faces = {k: v for k, v in pending_faces.items() if now - v['last_seen'] < PENDING_TIMEOUT}

        faces = app.get(frame)

        # Update prev_bboxes for next frame using current detections
        prev_bboxes = [
            expand_bbox(face.bbox, BBOX_MARGIN, frame.shape)
            for face in faces
        ]

        if faces:
            #shared_array[6] = 1.0
            for face in faces:
                emb = face.normed_embedding
                box = face.bbox.astype(int)

                label = "Unknown"
                color = (0, 0, 255)

                # ── Step 1: match against temp embeddings ─────────────────
                best_temp = None
                best_temp_sim = 0.0
                for track_id, data in temp_faces.items():
                    sim = float(np.dot(emb, data['embedding']))
                    if sim > best_temp_sim:
                        best_temp_sim = sim
                        best_temp = track_id
                print(f"temp sim: {best_temp_sim}")

                if best_temp and best_temp_sim >= TEMP_MATCH_THRESHOLD:
                    temp_faces[best_temp]['embedding'] = emb
                    temp_faces[best_temp]['frame_count'] += 1
                    temp_faces[best_temp]['last_seen'] = now

                    name = temp_faces[best_temp]['name']
                    if (name
                            and name in known_faces
                            and temp_faces[best_temp]['frame_count'] % GALLERY_UPDATE_INTERVAL == 0):
                        updated, was_updated = try_add_to_gallery(emb, known_faces[name])
                        if was_updated:
                            known_faces[name] = updated
                            if name in npy_faces:
                                save_gallery(name, updated)
                            print(f"{name}: gallery updated ({len(updated)} embeddings)")

                    label = name if name else f"pending ({best_temp[:4]})"
                    color = (0, 255, 0) if name else (0, 165, 255)

                elif best_temp and best_temp_sim >= 0.3:
                    # Low confidence but plausible — keep temp fresh without
                    # incrementing frame_count so gallery updates don't trigger
                    temp_faces[best_temp]['embedding'] = emb
                    temp_faces[best_temp]['last_seen'] = now
                    label = temp_faces[best_temp]['name'] or f"pending ({best_temp[:4]})"
                    color = (0, 255, 0) if temp_faces[best_temp]['name'] else (0, 165, 255)

                # ── Step 2: no temp match — check gallery ─────────────────
                else:
                    best_gallery_name = None
                    best_gallery_sim = 0.0
                    best_gallery_emb = None
                    for name, gallery in known_faces.items():
                        sim, idx = match_gallery(emb, gallery)
                        if sim > best_gallery_sim:
                            best_gallery_sim = sim
                            best_gallery_name = name
                            best_gallery_emb = gallery[idx]
                    print(f"gallery sim: {best_gallery_sim}")

                    if best_gallery_name and best_gallery_sim >= GALLERY_MATCH_THRESHOLD:
                        track_id = uuid.uuid4().hex[:8]
                        temp_faces[track_id] = {
                            'embedding': best_gallery_emb,
                            'name': best_gallery_name,
                            'frame_count': 1,
                            'last_seen': now
                        }
                        print(f"Gallery match: {best_gallery_name} ({best_gallery_sim:.2f}) → temp {track_id}")
                        label = best_gallery_name
                        color = (0, 255, 0)

                    # ── Step 3: check/create pending ──────────────────────
                    else:
                        best_pending = None
                        best_pending_sim = 0.0
                        for temp_id, data in pending_faces.items():
                            sim, _ = match_gallery(emb, data['gallery'])
                            if sim > best_pending_sim:
                                best_pending_sim = sim
                                best_pending = temp_id
                        print(f"pending sim: {best_pending_sim}")

                        if best_pending and best_pending_sim >= CONFIRM_THRESHOLD:
                            pending_faces[best_pending]['confirmations'] += 1
                            pending_faces[best_pending]['last_seen'] = now
                            pending_faces[best_pending]['gallery'], _ = try_add_to_gallery(
                                emb, pending_faces[best_pending]['gallery'], pending=True
                            )
                            confs = pending_faces[best_pending]['confirmations']
                            print(f"Pending {best_pending}: {confs}/{CONFIRMATIONS_NEEDED} confirmations")

                            label = f"pending {confs}/{CONFIRMATIONS_NEEDED}"
                            color = (0, 165, 255)

                            if confs >= CONFIRMATIONS_NEEDED:
                                new_name = f"unknown_{uuid.uuid4().hex[:8]}"
                                known_faces[new_name] = pending_faces.pop(best_pending)['gallery']
                                npy_faces.add(new_name)
                                save_gallery(new_name, known_faces[new_name])
                                print(f"Face confirmed and saved as: {new_name}")
                        else:
                            temp_id = uuid.uuid4().hex[:8]
                            pending_faces[temp_id] = {
                                'gallery': [emb],
                                'confirmations': 1,
                                'last_seen': now
                            }
                            print(f"New pending face: {temp_id}")
                            label = "Unknown"
                            color = (0, 0, 255)

                # Save reference face crop when similarity is very high
                if best_temp_sim > 0.92:
                    x1, y1, x2, y2 = expand_bbox(face.bbox, 0.1, frame.shape)
                    face_crop = frame[y1:y2, x1:x2]
                    if face_crop.size > 0:
                        ref_path = os.path.join(references_dir, f"ref_{uuid.uuid4().hex[:8]}.jpg")
                        cv2.imwrite(ref_path, face_crop)
                        references.append(face_crop)
                        print(f"Reference saved: {ref_path} ({len(references)} total)")

                # ── Draw bounding box and label ────────────────────────────
                cv2.rectangle(frame, (box[0], box[1]), (box[2], box[3]), color, 2)
                label_size, _ = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, 0.6, 2)
                cv2.rectangle(frame, (box[0], box[1] - label_size[1] - 10),
                              (box[0] + label_size[0], box[1]), color, -1)
                cv2.putText(frame, label, (box[0], box[1] - 5),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)

                if time.time() - timestamp > 0.1:
                    timestamp = time.time()

        else:
            #shared_array[6] = 0.0
            prev_bboxes = []  # no faces — clear bboxes so we don't correct stale regions
            print("no face detected")

        cv2.imshow("Face Recognition", frame)
        if cv2.waitKey(1) == 27:
            break

def camera_process_2() -> None:
    """Process running the camera / vision logic."""
    print("Loading model...")
    app = FaceAnalysis(
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
    app.prepare(ctx_id=0, det_size=(640, 640))

    known_faces = {}
    npy_faces = set()
    images_dir = 'AI/images'
    embeddings_dir = 'AI/images/embeddings'
    references_dir = 'captured_stream'

    # Load reference face crops for histogram matching
    # These should be face crops (not full frames) taken under ideal lighting
    references = []
    os.makedirs(references_dir, exist_ok=True)
    filenames = sorted(os.listdir(references_dir))
    for filename in filenames:
        if filename.endswith(('.jpg', '.png')):
            ref = cv2.imread(os.path.join(references_dir, filename))
            if ref is not None:
                references.append(ref)
                #print(f"Loaded reference: {filename}")
    print(f"Loaded {len(references)} reference images")

    # print("Loading known faces from images...")
    # for filename in ['robin.jpg', 'thomas.jpg', 'jorien.jpg', 'Wannes.jpg']:
    #     image_path = os.path.join(images_dir, filename)
    #     if os.path.exists(image_path):
    #         img = cv2.imread(image_path)
    #         if img is not None:
    #             faces = app.get(img)
    #             if len(faces) > 0:
    #                 name = os.path.splitext(filename)[0]
    #                 known_faces[name] = [faces[0].normed_embedding]
    #                 print(f"Loaded face: {name}")
    #             else:
    #                 print(f"No face detected in {filename}")
    #         else:
    #             print(f"Could not read {filename}")
    #     else:
    #         print(f"File not found: {image_path}")

    # os.makedirs(embeddings_dir, exist_ok=True)
    # for filename in os.listdir(embeddings_dir):
    #     if filename.endswith('.npz'):
    #         name = os.path.splitext(filename)[0]
    #         if name not in known_faces:
    #             known_faces[name] = load_gallery(os.path.join(embeddings_dir, filename))
    #             npy_faces.add(name)
    #             print(f"Loaded gallery: {name} ({len(known_faces[name])} embeddings)")

    # print(f"Loaded {len(known_faces)} known faces")
    # print("Starting webcam...")

    # cap = cv2.VideoCapture(0)
    # cam_info_faces = initialize_coordinate_detection_faces()

    # M = load_matrix()

    # shared_array[10] = 1.0
    # timestamp = time.time()

    # temp_faces = {}
    # pending_faces = {}

    # # Stores bounding boxes from the previous frame
    # # { track_id -> expanded_bbox (x1, y1, x2, y2) }
    # prev_bboxes = []

    emb = []

    for frame in references:

        faces = app.get(frame)
        for face in faces:
                emb.append(face.normed_embedding)

    for i in range(len(emb)-1):
        sim = float(np.dot(emb[i] , emb[i+1]))
        print(sim)


def deform_stream():
    references_dir = 'AI/images/references'
    original_stream_dir = 'debug/captured_stream'
    deformed_stream_dir = 'debug/deformed_stream'

    # Load reference face crops for histogram matching
    # These should be face crops (not full frames) taken under ideal lighting
    references = []
    os.makedirs(references_dir, exist_ok=True)
    for filename in os.listdir(references_dir):
        if filename.endswith(('.jpg', '.png')):
            ref = cv2.imread(os.path.join(references_dir, filename))
            if ref is not None:
                faces = app.get(ref)
                for face in faces:
                    x1, y1, x2, y2 = face.bbox
                    references.append(ref[x1 : y1, x2: y2])
                print(f"Loaded reference: {filename}")
    print(f"Loaded {len(references)} reference images")

    stream = []
    os.makedirs(original_stream_dir, exist_ok=True)
    for filename in os.listdir(original_stream_dir):
        if filename.endswith(('.jpg', '.png')):
            ref = cv2.imread(os.path.join(original_stream_dir, filename))
            if ref is not None:
                faces = app.get(ref)
                for face in faces:
                    x1, y1, x2, y2 = face.bbox
                    stream.append(ref[x1 : y1, x2: y2])
                print(f"Loaded reference: {filename}")
    print(f"Loaded {len(stream)} stream images")

    for frame in streams:

        faces = app.get(frame)
            for face in faces:
                frame = correct_face_region(frame, bbox, references)
                os.makedirs(deformed_stream_dir, exist_ok=True)
                im_path = os.path.join(save_path, f"ref_{time.time()}.png")
                cv2.imwrite(im_path, frame)




camera_process_2()