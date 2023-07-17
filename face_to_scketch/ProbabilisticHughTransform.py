import cv2
import dlib
import json
import numpy as np
import math

def find_closest_point(points, target_point):
    closest_point = None
    min_distance = float('inf')

    for i, point in enumerate(points):
        distance = math.sqrt((point[0] - target_point[0])**2 + (point[1] - target_point[1])**2)
        if distance < min_distance:
            min_distance = distance
            closest_point = i

    return closest_point, min_distance

detector = dlib.get_frontal_face_detector()
predictor = dlib.shape_predictor('data/shape_predictor_68_face_landmarks.dat')
face_cascade = cv2.CascadeClassifier(cv2.data.haarcascades + 'haarcascade_frontalface_default.xml')

cap = cv2.VideoCapture(0)

def scale_image(image, target_size):
    height, width = image.shape[:2]

    # Calculate the scaling factors for width and height
    scale_w = target_size[0] / width
    scale_h = target_size[1] / height

    # Choose the appropriate scaling factor
    scale = min(scale_w, scale_h)

    # Resize the image based on the chosen scale
    resized_image = cv2.resize(image, None, fx=scale, fy=scale, interpolation=cv2.INTER_CUBIC)

    return resized_image

def joined_segments(segments, img=None):
    maximum_dist = 10

    final_lines = []

    new_segments = []
    for seg in segments:
        new_segments.append([int(seg[0]), int(seg[1])])
        new_segments.append([int(seg[2]), int(seg[3])])
    segments = new_segments.copy()
    del new_segments

    initial_seg_count = len(segments)

    while segments:

        new_line = [segments.pop(0), segments.pop(0)]
        
        while True:

            if not segments:
                break

            closest_point_0, min_dist_0 = find_closest_point(segments, new_line[0])
            closest_point_1, min_dist_1 = find_closest_point(segments, new_line[-1])
            min_dist = min_dist_0 if min_dist_0 < min_dist_1 else min_dist_1
            closest_point = closest_point_0 if min_dist_0 < min_dist_1 else closest_point_1

            if min_dist > maximum_dist:
                break
            
            if closest_point%2 == 0:
                seg2 = segments.pop(closest_point+1)
                seg1 = segments.pop(closest_point)
            else:
                seg1 = segments.pop(closest_point)
                seg2 = segments.pop(closest_point-1)
            
            if min_dist == min_dist_0:
                new_line = [seg1, seg2] + new_line
            else:
                new_line.append(seg1)
                new_line.append(seg2)

            height, width = img.shape
            lines_visual = np.zeros((height, width, 3), dtype=np.uint8)

            for line in final_lines:
                random_color = [np.random.random()*255, np.random.random()*255, np.random.random()*255]
                for i in range(len(line)-1):
                    cv2.line(lines_visual, (line[i][0], line[i][1]), (line[i+1][0], line[i+1][1]), (random_color[0], random_color[1], random_color[2]), 2)

            for i in range(len(new_line)-1):
                cv2.line(lines_visual, (new_line[i][0], new_line[i][1]), (new_line[i+1][0], new_line[i+1][1]), (0, 255, 0), 2)

            cv2.imshow('drawing', lines_visual)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

        final_lines.append(new_line)

        print("Average line length: ", np.round(sum([len(line) for line in final_lines])/len(final_lines), 2))
        print(f"Progress status: {((initial_seg_count-len(segments))*100/initial_seg_count):.2f}% done")

    return final_lines

def plot_lines(lines, image):

    for line in lines:
        random_color = [np.random.random()*255, np.random.random()*255, np.random.random()*255]
        for i in range(len(line)-1):
            cv2.line(image, (line[i][0], line[i][1]), (line[i+1][0], line[i+1][1]), (random_color[0], random_color[1], random_color[2]), 2)
    return image

while True:
    faces = []
    ret, image = cap.read()

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
    image = scale_image(image, [800, 800])

    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

    gray = cv2.GaussianBlur(gray, (5, 5), 0)

    binary = cv2.adaptiveThreshold(gray, 255, cv2.ADAPTIVE_THRESH_MEAN_C, cv2.THRESH_BINARY, 11, 2)

    image = np.repeat(binary[:, :, np.newaxis], 3, axis=2)

    src = cv2.bitwise_not(binary)

    ret, binary_map = cv2.threshold(src,127,255,0)

    nlabels, labels, stats, centroids = cv2.connectedComponentsWithStats(binary_map, None, None, None, 8, cv2.CV_32S)

    areas = stats[1:,cv2.CC_STAT_AREA]

    result = np.zeros((labels.shape), np.uint8)

    for i in range(0, nlabels - 1):
        if areas[i] >= 100:   #keep
            result[labels == i + 1] = 255

    img = cv2.bitwise_not(result)

    minLineLength = 2
    maxLineGap = 2

    # apply probabilistic Hough transform
    lines = cv2.HoughLinesP(cv2.bitwise_not(img), 1, np.pi/180, 5, minLineLength, maxLineGap)
    lines = [line[0] for line in lines]
    lines = [[point for point in line] for line in lines]

    height, width = img.shape
    final = np.zeros((height, width, 3), dtype=np.uint8)

    print("Calculating lines...")

    lines = joined_segments(lines, img)
    final = plot_lines(lines, final)

    """
    try:
        print(lines.shape)
        for line in lines:
            for x1, y1, x2, y2 in line:
                print(line)
                cv2.line(final, (x1, y1), (x2, y2), (0, 255, 0), 2)
    except:
        print("error")
    """

    json.dump(lines, open('data/good_drawing.json', 'w'))
    #cv2.imwrite("data/warhol.jpg", result)
    cv2.imshow('drawing finished', final)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# Release the video capture and close the window
cap.release()
cv2.destroyAllWindows()

