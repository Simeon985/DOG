from processes.camera.utils import *


TEMP_MATCH_THRESHOLD = 0.4
GALLERY_MATCH_THRESHOLD = 0.4
GALLERY_UPDATE_INTERVAL = 10
TEMP_TIMEOUT = 2.0
CONFIRMATIONS_NEEDED = 5
PENDING_TIMEOUT = 10.0
CONFIRM_THRESHOLD = 0.6


def camera_process(shared_array: SynchronizedArray) -> None:
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

    cap = cv2.VideoCapture(gstreamer_pipeline(flip_method=2))
    cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
    cam_info_faces = initialize_coordinate_detection_faces()

    M = load_matrix()

    shared_array[10] = 1.0
    timestamp = time.time()

    temp_faces = {}
    pending_faces = {}

    while True:
        ret, frame = cap.read()
        if not ret:
            break
        if M is not None:
            frame = apply_matrix(frame, M)
        now = time.time()

        temp_faces    = {k: v for k, v in temp_faces.items()    if now - v['last_seen'] < TEMP_TIMEOUT}
        pending_faces = {k: v for k, v in pending_faces.items() if now - v['last_seen'] < PENDING_TIMEOUT}

        faces = app.get(frame)
        if faces:
            shared_array[6] = 1.0
            for face in faces:
                emb = face.normed_embedding
                box = face.bbox.astype(int)

                # These will be set based on match result for drawing
                label = "Unknown"
                color = (0, 0, 255)  # red

                # ── Step 1: match against temp embeddings ──────────────────────
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
                    color = (0, 255, 0) if name else (0, 165, 255)  # green if known, orange if pending

                # ── Step 2: no temp match — check gallery ──────────────────────
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
                        color = (0, 255, 0)  # green

                    # ── Step 3: no temp or gallery match — check/create pending ──
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
                            color = (0, 165, 255)  # orange

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
                            color = (0, 0, 255)  # red

                # ── Draw bounding box and label ────────────────────────────────
                cv2.rectangle(frame, (box[0], box[1]), (box[2], box[3]), color, 2)
                label_size, _ = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, 0.6, 2)
                cv2.rectangle(frame, (box[0], box[1] - label_size[1] - 10),
                              (box[0] + label_size[0], box[1]), color, -1)
                cv2.putText(frame, label, (box[0], box[1] - 5),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
                score = sharpness_score(frame, face.bbox)
                print(f"score: {score}")
                cv2.putText(frame, f"sharp: {score:.0f}", (box[0], box[3] + 20),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 0), 2)

                if time.time() - timestamp > 0.1:
                    timestamp = time.time()

        else:
            shared_array[6] = 0.0
            print("no face detected")




        # for face in faces:
        #     emb = face.normed_embedding
        #     box = face.bbox.astype(int)
            
        #     # Compare with known faces
        #     best_match = None
        #     best_similarity = 0
            
        #     for name, known_emb in known_faces.items():
        #         # Calculate cosine similarity
        #         similarity = np.dot(emb, known_emb)
        #         if similarity > best_similarity:
        #             best_similarity = similarity
        #             best_match = name
            
        #     # Draw bounding box and label
        #     if best_match and best_similarity >= threshold:
        #         color = (0, 255, 0)  # Green for recognized
        #         label = f"{best_match} ({best_similarity:.2f})"
        #     else:
        #         color = (0, 0, 255)  # Red for unknown
        #         label = "Unknown"
            
        #     cv2.rectangle(frame, (box[0], box[1]), (box[2], box[3]), color, 2)
            
        #     # Draw label
        #     label_size, _ = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, 0.6, 2)
        #     cv2.rectangle(frame, (box[0], box[1] - label_size[1] - 10), 
        #                 (box[0] + label_size[0], box[1]), color, -1)
        #     cv2.putText(frame, label, (box[0], box[1] - 5), 
        #             cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
        # # Draw fps counter
        # # cv2.putText(frame, f"FPS: {fps:.1f}", (10, 30),
        # #     cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
        cv2.imshow("Face Recognition", frame) 
        if cv2.waitKey(1) == 27:  # ESC to exit
            break

    cap.release()
    cv2.destroyAllWindows()