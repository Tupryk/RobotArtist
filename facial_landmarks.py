import cv2
import dlib
import json
import numpy as np


detector = dlib.get_frontal_face_detector()
predictor = dlib.shape_predictor('../shape_predictor_68_face_landmarks.dat')
cap = cv2.VideoCapture(0)

face_lines = json.load(open("data/face_lines.json"))

while True:
    
    ret, webcam_image = cap.read()
    image = np.zeros_like(webcam_image)
    gray = cv2.cvtColor(webcam_image, cv2.COLOR_BGR2GRAY)
    faces = detector(gray, 1)

    if len(faces) > 0:

        landmarks = predictor(gray, faces[0])

        real_lines = []
        for line in face_lines:
            real_line = []

            for i in range(len(line)):
                if i < len(line)-1:
                    cv2.line(image, (landmarks.part(line[i]).x, landmarks.part(line[i]).y),
                        (landmarks.part(line[i+1]).x, landmarks.part(line[i+1]).y), (0, 0, 255), 1)
                real_line.append([landmarks.part(line[i]).x, -landmarks.part(line[i]).y])
            real_lines.append(real_line)

        json.dump(real_lines, open('data/output.json', 'w'))
        cv2.imwrite("data/output.jpg", image)
        break

    if cv2.waitKey(10) == 27:
        break

cap.release()
