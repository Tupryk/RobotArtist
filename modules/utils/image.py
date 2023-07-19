import cv2
import numpy as np
import matplotlib.pyplot as plt


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

def get_drawing_bitmap(image, show=False):

    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

    gray = cv2.GaussianBlur(gray, (7, 7), 0)

    binary = cv2.adaptiveThreshold(gray, 255, cv2.ADAPTIVE_THRESH_MEAN_C, cv2.THRESH_BINARY, 11, 2)

    binary = cv2.bitwise_not(binary)
    nlabels, labels, stats, _ = cv2.connectedComponentsWithStats(binary, None, None, None, 8, cv2.CV_32S)
    areas = stats[1:,cv2.CC_STAT_AREA]
    result = np.zeros((labels.shape), np.uint8)
    for i in range(0, nlabels - 1):
        if areas[i] >= 100:   #keep
            result[labels == i + 1] = 255
    image = cv2.bitwise_not(result)

    if show:
        plt.imshow(result)
        plt.show()

    return image

def plot_lines(lines, image, show=False):
    for line in lines:
        random_color = [np.random.random()*255, np.random.random()*255, np.random.random()*255]
        for i in range(len(line)-1):
            cv2.line(image, (line[i][0], line[i][1]), (line[i+1][0], line[i+1][1]), (random_color[0], random_color[1], random_color[2]), 2)
    return image
