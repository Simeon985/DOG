import argus
import numpy as np
import cv2

# Create an Argus camera object
camera = argus.Camera()

# Set the camera settings
camera.set_mode(argus.CameraMode.CSI)

# Capture an image
image = camera.capture()

# Convert the image to a numpy array
image_array = np.array(image)

# Save numpy array as image
cv2.imwrite("output.jpg", your_array)
