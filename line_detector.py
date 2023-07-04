import cv2
import numpy as np

image = cv2.imread('data/warhol.jpg')

edges = cv2.Canny(image, 10, 200, apertureSize=3)

# Perform Probabilistic Hough Line Transform
lines = cv2.HoughLinesP(edges, rho=1, theta=np.pi / 180, threshold=100, minLineLength=100, maxLineGap=10)

if lines is not None:
    for line in lines:
        x1, y1, x2, y2 = line[0]
        cv2.line(image, (x1, y1), (x2, y2), (0, 0, 255), 2)

cv2.imwrite("data/lines.jpg", image)
