import cv2
import dlib
import json
import numpy as np


detector = dlib.get_frontal_face_detector()
predictor = dlib.shape_predictor('../shape_predictor_68_face_landmarks.dat')
cap = cv2.VideoCapture(0)

face_lines = json.load(open("face_lines.json"))

while True:
    
    ret, webcam_image = cap.read()
    image = np.zeros_like(webcam_image)
    gray = cv2.cvtColor(webcam_image, cv2.COLOR_BGR2GRAY)
    faces = detector(gray, 1)

    if len(faces) > 0:

        landmarks = predictor(gray, faces[0])

        points = []
        for line in face_lines:
            cv2.line(image, (landmarks.part(line[0]).x, landmarks.part(line[0]).y),
                     (landmarks.part(line[1]).x, landmarks.part(line[1]).y), (0, 0, 255), 1)
            points.append([
                [landmarks.part(line[0]).x, landmarks.part(line[0]).y],
                [landmarks.part(line[1]).x, landmarks.part(line[1]).y]
            ])

        json.dump(points, open('output.json', 'w'))
        cv2.imwrite("output.jpg", image)
        break

    if cv2.waitKey(10) == 27:
        break

cap.release()
