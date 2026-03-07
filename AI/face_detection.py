import cv2
import os

def detect_and_draw_faces(image_path, model_path="face_detection_yunet_2023mar.onnx"):
    # Check if model file exists
    if not os.path.exists(model_path):
        print(f"Model file not found: {model_path}")
        print("Please download from: https://github.com/opencv/opencv_zoo/blob/main/models/face_detection_yunet/face_detection_yunet_2023mar.onnx")
        return None
    
    # Read image
    img = cv2.imread(image_path)
    if img is None:
        print(f"Could not read image: {image_path}")
        return None
    
    # Make a copy for drawing
    img_with_boxes = img.copy()
    
    # Initialize detector with model file
    detector = cv2.FaceDetectorYN.create(
        model=model_path,
        config="",
        input_size=(320, 320),
        score_threshold=0.5,
        nms_threshold=0.3,
        top_k=5000
    )
    
    # Set input size to image size
    detector.setInputSize((img.shape[1], img.shape[0]))
    
    # Detect faces
    _, faces = detector.detect(img)
    
    # Draw rectangles around faces
    if faces is not None:
        for face in faces:
            # Get face coordinates (x, y, width, height)
            x, y, w, h = map(int, face[:4])
            
            # Draw rectangle
            cv2.rectangle(img_with_boxes, (x, y), (x + w, y + h), (0, 255, 0), 2)
            
            # Add confidence score (optional)
            confidence = face[-1]
            cv2.putText(img_with_boxes, f"{confidence:.2f}", (x, y-10), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
        
        print(f"Found {len(faces)} faces")
    else:
        print("No faces found")
    
    return img_with_boxes

# Usage
result_image = detect_and_draw_faces("image.jpg")

if result_image is not None:
    # Show image
    cv2.imshow("Face Detection", result_image)
    cv2.waitKey(0)
    cv2.destroyAllWindows()
    
    # Save image
    cv2.imwrite("output_with_faces.jpg", result_image)
    print("Output saved as output_with_faces.jpg")