import cv2
import numpy as np
from robotic import ry
import matplotlib.pyplot as plt


def look_path(center=np.array([0, 0, 1.5]), radious=np.array([-.6, 0, 0]), count_=6, ry_config=None):
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

def search_faces(ry_config, bot):

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
        bot.move(komo.getPath(), [1.])
        while bot.getTimeToEnd() > 0:
            bot.sync(ry_config, .1)

        image, _ = bot.getImageAndDepth("r_gripperCamera")

        face = get_face_from_image(image)
        if face:
            print("Found a face!")
            plt.imshow(image)
            plt.show()
            return face_to_sketch(face)
        
        # Restart point loop if necessary
        index += 1
        if index >= len(points): index = 0
