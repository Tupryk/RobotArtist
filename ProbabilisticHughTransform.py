import cv2
import dlib
import numpy as np

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

def joined_segments(segments):
    maximum_dist = 10
    final_lines = []
    total_lines = segments.shape[0]
    for i, seg in enumerate(segments):

        try:
            segments = np.delete(segments, i, axis=0)
            seg = np.array([[seg[0][0], seg[0][1]], [seg[0][2], seg[0][3]]])
            line = [seg[0].tolist(), seg[1].tolist()]

            for j, seg_comp in enumerate(segments):
                seg_comp = np.array([[seg_comp[0][0], seg_comp[0][1]], [seg_comp[0][2], seg_comp[0][3]]])
                if np.linalg.norm(seg[1]-seg_comp[0]) <= maximum_dist:
                    segments = np.delete(segments, j, axis=0)
                    seg_comp[0] = seg[1]
                    line.append(seg_comp[0].tolist())
            
            final_lines.append(line)
        except:
            pass

        if final_lines:
            print(f"Average line size: {sum([len(l) for l in final_lines])/len(final_lines)}")
            print(f"Created {len(final_lines)} lines. Total segments {total_lines}")

    print(final_lines)
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

    height, width = img.shape
    final = np.zeros((height, width, 3), dtype=np.uint8)

    lines = joined_segments(lines)
    final = plot_lines(lines, final)

    try:
        """
        print(lines.shape)
        for line in lines:
            for x1, y1, x2, y2 in line:
                print(line)
                cv2.line(final, (x1, y1), (x2, y2), (0, 255, 0), 2)
        """
    except:
        print("error")

    #cv2.imwrite("data/warhol.jpg", result)
    cv2.imshow('Webcam', final)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# Release the video capture and close the window
cap.release()
cv2.destroyAllWindows()

