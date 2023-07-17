import cv2
import numpy as np


def get_face(image):
    face_cascade = cv2.CascadeClassifier(cv2.data.haarcascades + 'haarcascade_frontalface_default.xml')

    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

    faces = face_cascade.detectMultiScale(gray, scaleFactor=1.1, minNeighbors=5, minSize=(30, 30))

    if len(faces) > 0:
        (x, y, w, h) = faces[0]
        
        margin = 20
        x -= margin
        y -= margin
        w += 2 * margin
        h += 2 * margin

        mask = np.zeros(image.shape[:2], dtype=np.uint8)
        cv2.rectangle(mask, (x, y), (x + w, y + h), 255, -1)

        return cv2.bitwise_and(image, image, mask=mask)
    else:
        return None

def face_to_sketch(image):
    return []
