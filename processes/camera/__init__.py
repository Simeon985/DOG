from processes.camera.utils import *


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

    # known_faces: { name -> [emb1, emb2, ...] }
    known_faces = {}
    npy_faces = set()  # names that came from .npz files (should be saved back to disk)
    images_dir = 'AI/images'
    embeddings_dir = 'AI/images/embeddings'
    MATCH_THRESHOLD = 0.4       # min similarity to count as a match
    CONFIRM_THRESHOLD = 0.6     # min similarity to confirm a pending face
    CONFIRMATIONS_NEEDED = 5
    PENDING_TIMEOUT = 10.0

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

    # Load previously saved galleries
    os.makedirs(embeddings_dir, exist_ok=True)
    for filename in os.listdir(embeddings_dir):
        if filename.endswith('.npz'):
            name = os.path.splitext(filename)[0]
            if name not in known_faces:
                known_faces[name] = load_gallery(os.path.join(embeddings_dir, filename))
                npy_faces.add(name)
                print(f"Loaded saved gallery: {name} ({len(known_faces[name])} embeddings)")

    print(f"Loaded {len(known_faces)} known faces")
    print("Starting webcam...")

    cap = cv2.VideoCapture(gstreamer_pipeline(flip_method=0))
    cam_info_faces = initialize_coordinate_detection_faces()

    shared_array[10] = 1.0
    timestamp = time.time()

    # pending_faces: { temp_id -> { 'gallery': [emb, ...], 'confirmations': int, 'last_seen': float } }
    pending_faces = {}

    while True:
        ret, frame = cap.read()
        if not ret:
            break

        # Clean up expired pending faces
        now = time.time()
        pending_faces = {
            k: v for k, v in pending_faces.items()
            if now - v['last_seen'] < PENDING_TIMEOUT
        }

        faces = app.get(frame)
        if faces:
            shared_array[6] = 1.0
            for face in faces:
                emb = face.normed_embedding

                # Match against confirmed known faces
                best_match = None
                best_similarity = 0.0
                for name, gallery in known_faces.items():
                    similarity = match_gallery(emb, gallery)
                    if similarity > best_similarity:
                        best_similarity = similarity
                        best_match = name

                if best_match and best_similarity >= MATCH_THRESHOLD:
                    # Matched — try to add to gallery if diverse and confident enough
                    updated = update_gallery(emb, known_faces[best_match], best_similarity)
                    if len(updated) > len(known_faces[best_match]):
                        known_faces[best_match] = updated
                        print(f"{best_match}: gallery updated ({len(updated)} embeddings)")
                        if best_match in npy_faces:
                            save_gallery(best_match, known_faces[best_match])

                else:
                    # Match against pending faces
                    best_pending = None
                    best_pending_similarity = 0.0
                    for temp_id, data in pending_faces.items():
                        similarity = match_gallery(emb, data['gallery'])
                        if similarity > best_pending_similarity:
                            best_pending_similarity = similarity
                            best_pending = temp_id

                    if best_pending and best_pending_similarity >= CONFIRM_THRESHOLD:
                        pending_faces[best_pending]['confirmations'] += 1
                        pending_faces[best_pending]['last_seen'] = now
                        pending_faces[best_pending]['gallery'] = update_gallery(
                            emb,
                            pending_faces[best_pending]['gallery'],
                            best_pending_similarity
                        )

                        print(f"Pending {best_pending}: "
                              f"{pending_faces[best_pending]['confirmations']}/{CONFIRMATIONS_NEEDED} confirmations, "
                              f"{len(pending_faces[best_pending]['gallery'])} embeddings")

                        if pending_faces[best_pending]['confirmations'] >= CONFIRMATIONS_NEEDED:
                            # Promoted to known face
                            new_name = f"unknown_{uuid.uuid4().hex[:8]}"
                            known_faces[new_name] = pending_faces.pop(best_pending)['gallery']
                            npy_faces.add(new_name)
                            save_gallery(new_name, known_faces[new_name])
                            print(f"Face confirmed and saved as: {new_name} "
                                  f"({len(known_faces[new_name])} embeddings)")
                    else:
                        # Brand new — start a pending entry with a single-embedding gallery
                        temp_id = uuid.uuid4().hex[:8]
                        pending_faces[temp_id] = {
                            'gallery': [emb],
                            'confirmations': 1,
                            'last_seen': now
                        }
                        print(f"New pending face: {temp_id}")

                #x, y, z = calculate_3D_coordinates_faces(face, cam_info_faces)
                if time.time() - timestamp > 0.1:
                    #print(f'{x}, {y}, {z}')
                    timestamp = time.time()
        else:
            shared_array[6] = 0.0
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
        # cv2.imshow("Face Recognition", frame) 
        if cv2.waitKey(1) == 27:  # ESC to exit
            break

    cap.release()
    cv2.destroyAllWindows()