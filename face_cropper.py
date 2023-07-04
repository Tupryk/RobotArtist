import cv2
import numpy as np

# Load the Haar cascade classifier for face detection
face_cascade = cv2.CascadeClassifier(cv2.data.haarcascades + 'haarcascade_frontalface_default.xml')

# Load the image
image = cv2.imread('data/lex.jpg')

# Convert the image to grayscale
gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

# Detect faces in the grayscale image
faces = face_cascade.detectMultiScale(gray, scaleFactor=1.1, minNeighbors=5, minSize=(30, 30))

# Extract the first detected face
if len(faces) > 0:
    (x, y, w, h) = faces[0]
    
    # Add some margin to the face bounding box
    margin = 20
    x -= margin
    y -= margin
    w += 2 * margin
    h += 2 * margin

    # Create a mask with white background
    mask = np.zeros(image.shape[:2], dtype=np.uint8)
    cv2.rectangle(mask, (x, y), (x + w, y + h), 255, -1)

    # Bitwise-AND the mask with the original image
    result = cv2.bitwise_and(image, image, mask=mask)

    cv2.imwrite("data/output.jpg", result)
else:
    print("No faces found in the image.")
