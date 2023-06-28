import cv2
import dlib
import json
import numpy as np
from time import sleep


detector = dlib.get_frontal_face_detector()
predictor = dlib.shape_predictor('data/shape_predictor_68_face_landmarks.dat')
cap = cv2.VideoCapture(0)

face_lines = json.load(open("data/face_lines.json"))

sleep(3)
face_cascade = cv2.CascadeClassifier(cv2.data.haarcascades + 'haarcascade_frontalface_default.xml')

while True:
    faces = []
    ret, image = cap.read()
    """
    while not len(faces):
        ret, image = cap.read()

        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        faces = face_cascade.detectMultiScale(gray, scaleFactor=1.1, minNeighbors=5, minSize=(30, 30))

    for (x, y, w, h) in faces:
        roi_x = max(0, x - int(w * 0.25))
        roi_y = max(0, y - int(h * 0.35))
        roi_w = min(image.shape[1] - roi_x, int(w * 1.5))
        roi_h = min(image.shape[0] - roi_y, int(h * 1.5))
        break

    cv2.rectangle(image, (roi_x, roi_y), (roi_x+roi_w, roi_y+roi_h), (0, 255, 0), 2)

    image = image[roi_y:roi_y+roi_h, roi_x:roi_x+roi_w]
    """

    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

    gray = cv2.GaussianBlur(gray, (5, 5), 0)

    binary = cv2.adaptiveThreshold(gray, 255, cv2.ADAPTIVE_THRESH_MEAN_C, cv2.THRESH_BINARY, 11, 2)

    image = np.repeat(binary[:, :, np.newaxis], 3, axis=2)

    #out_binary = cv2.threshold(image, 0, 255, cv2.THRESH_OTSU )[1]

    src = cv2.bitwise_not(binary)

    ret, binary_map = cv2.threshold(src,127,255,0)

    nlabels, labels, stats, centroids = cv2.connectedComponentsWithStats(binary_map, None, None, None, 8, cv2.CV_32S)

    areas = stats[1:,cv2.CC_STAT_AREA]

    result = np.zeros((labels.shape), np.uint8)

    for i in range(0, nlabels - 1):
        if areas[i] >= 100:   #keep
            result[labels == i + 1] = 255

    result = cv2.bitwise_not(result)

    #cv2.imwrite("data/warhol.jpg", result)
    cv2.imshow('Webcam', result)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# Release the video capture and close the window
cap.release()
cv2.destroyAllWindows()
