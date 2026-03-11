import cv2
import numpy as np
from insightface.app import FaceAnalysis
import os

# Load model
app = FaceAnalysis(name='buffalo_l')
app.prepare(ctx_id=-1, det_size=(640, 640))

# Load known faces from images
known_faces = {}  # Dictionary to store name -> embedding
images_dir = 'images'
threshold = 0.6  # Similarity threshold for recognition

print("Loading known faces...")
for filename in ['robin.jpg', 'thomas.jpg', 'jorien.jpg', 'Wannes.jpg']:
    image_path = os.path.join(images_dir, filename)
    if os.path.exists(image_path):
        img = cv2.imread(image_path)
        if img is not None:
            faces = app.get(img)
            if len(faces) > 0:
                # Use the first detected face
                face = faces[0]
                name = os.path.splitext(filename)[0]  # Get name without extension
                known_faces[name] = face.normed_embedding
                print(f"Loaded face: {name}")
            else:
                print(f"No face detected in {filename}")
        else:
            print(f"Could not read {filename}")
    else:
        print(f"File not found: {image_path}")

print(f"Loaded {len(known_faces)} known faces")
print("Starting webcam...")

cap = cv2.VideoCapture(0)

while True:
    ret, frame = cap.read()
    if not ret:
        break
    
    faces = app.get(frame)

    for face in faces:
        emb = face.normed_embedding
        box = face.bbox.astype(int)
        
        # Compare with known faces
        best_match = None
        best_similarity = 0
        
        for name, known_emb in known_faces.items():
            # Calculate cosine similarity
            similarity = np.dot(emb, known_emb)
            if similarity > best_similarity:
                best_similarity = similarity
                best_match = name
        
        # Draw bounding box and label
        if best_match and best_similarity >= threshold:
            color = (0, 255, 0)  # Green for recognized
            label = f"{best_match} ({best_similarity:.2f})"
        else:
            color = (0, 0, 255)  # Red for unknown
            label = "Unknown"
        
        cv2.rectangle(frame, (box[0], box[1]), (box[2], box[3]), color, 2)
        
        # Draw label
        label_size, _ = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, 0.6, 2)
        cv2.rectangle(frame, (box[0], box[1] - label_size[1] - 10), 
                     (box[0] + label_size[0], box[1]), color, -1)
        cv2.putText(frame, label, (box[0], box[1] - 5), 
                   cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)

    cv2.imshow("Face Recognition", frame)
    if cv2.waitKey(1) == 27:  # ESC to exit
        break

cap.release()
cv2.destroyAllWindows()
