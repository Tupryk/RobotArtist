import cv2
import dlib
import json
import numpy as np
from robotic import ry
import matplotlib.pyplot as plt


def look_path(center=np.array([0, 0, 1.5]), radious=np.array([.6, 0, 0]), count_=6, ry_config=None):
    points = []
    starting_angle=np.pi/count_
    rotation_matrix = np.array([
        [np.cos(starting_angle), -np.sin(starting_angle), 0],
        [np.sin(starting_angle),  np.cos(starting_angle), 0],
        [                   0,                     0, 1],
    ])
    if ry_config:
        ry_config.addFrame("HeadCenter") \
            .setPosition(center) \
            .setShape(ry.ST.sphere, size=[.02]) \
            .setColor([0, 1, 0])
    radious = radious @ rotation_matrix
    radious = radious @ rotation_matrix
    for i in range(count_):
        radious = radious @ rotation_matrix

        if ry_config:
            ry_config.addFrame("Looking_point"+str(i)) \
                .setPosition(radious+center) \
                .setShape(ry.ST.sphere, size=[.02]) \
                .setColor([1, 0, 0])
        
        points.append(radious+center)
    return points

def get_face_from_image(image):

    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    face_cascade = cv2.CascadeClassifier(cv2.data.haarcascades + 'haarcascade_frontalface_default.xml')
    faces = face_cascade.detectMultiScale(gray, scaleFactor=1.1, minNeighbors=5, minSize=(30, 30))

    for (x, y, w, h) in faces:
        roi_x = max(0, x - int(w * 0.25))
        roi_y = max(0, y - int(h * 0.35))
        roi_w = min(image.shape[1] - roi_x, int(w * 1.5))
        roi_h = min(image.shape[0] - roi_y, int(h * 1.5))

        cv2.rectangle(image, (roi_x, roi_y), (roi_x+roi_w, roi_y+roi_h), (0, 255, 0), 2)

        image = image[roi_y:roi_y+roi_h, roi_x:roi_x+roi_w]

        return image, True

    return None, False

def face_to_sketch(image):
    return []

def face_to_landmarks(image):
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

    detector = dlib.get_frontal_face_detector()
    predictor = dlib.shape_predictor('data/shape_predictor_68_face_landmarks.dat')

    faces = detector(gray, 1)

    face_lines = json.load(open("data/face_lines.json"))

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

        return real_lines
    else:
        print("Failed capturing landmarks.")
        return None

def search_faces(ry_config, bot, simple=False):

    # Load search points
    points = look_path(ry_config=ry_config)
    index = 0
    while True:

        # Generate path for searching
        komo = ry.KOMO()
        komo.setConfig(ry_config, True)
        komo.setTiming(1, 3, 1., 2)
        komo.addControlObjective([], 0, 1e-2)
        komo.addControlObjective([], 2, 1e1)

        komo.addObjective([], ry.FS.accumulatedCollisions, [], ry.OT.eq)
        komo.addObjective([], ry.FS.jointLimits, [], ry.OT.ineq)

        komo.addObjective([], ry.FS.vectorZ, ["l_gripper"], ry.OT.sos, [2.], [-1, 0, 0])
        komo.addObjective([], ry.FS.vectorY, ["l_gripper"], ry.OT.eq, [0.5], [0, 0, 1])

        komo.addObjective([1.], ry.FS.position, ["pen_tip"], ry.OT.eq, [1e1], points[index])

        ry.NLP_Solver() \
            .setProblem(komo.nlp()) \
            .setOptions(stopTolerance=1e-2, verbose=0) \
            .solve()
        
        # Move robot throught generated path
        bot.move(komo.getPath(), [2.])
        while bot.getTimeToEnd() > 0:
            bot.sync(ry_config, .1)

        image, _ = bot.getImageAndDepth("l_gripperCamera")
        image = cv2.flip(image, 0)

        face, success = get_face_from_image(image)
        if success:
            print("Found a face!")
            plt.imshow(face)
            plt.show()
            if simple:
                lines = face_to_landmarks(face)
                if lines: return lines
            return face_to_sketch(face)
        
        # Restart point loop if necessary
        index += 1
        if index >= len(points): index = 0
