import numpy as np
import json
import cv2

def plot_lines(lines, image):

    for line in lines:
        random_color = [np.random.random()*255, np.random.random()*255, np.random.random()*255]
        for i in range(len(line)-1):
            cv2.line(image, (line[i][0], line[i][1]), (line[i+1][0], line[i+1][1]), (random_color[0], random_color[1], random_color[2]), 2)
    return image

def exterminate_extra_points(sketch, threshold=30):
    original_point_count = sum(len(line) for line in sketch)
    original_points_per_line = original_point_count/len(sketch)

    for line in sketch:
        for i in reversed(range(len(line)-1)):
            distance = np.sqrt((line[i][0] - line[i+1][0])**2 + (line[i][1] - line[i+1][1])**2)
            if distance < threshold:
                line.pop(i)

    new_point_count = sum(len(line) for line in sketch)
    new_points_per_line = new_point_count/len(sketch)

    print(f"Original point count: {original_point_count}")
    print(f"Original avg. points per line: {original_points_per_line}")
    print(f"New point count: {new_point_count}")
    print(f"New avg. points per line: {new_points_per_line}")

    return sketch

sketch = json.load(open("data/good_drawing.json"))

max_x = max(point[0] for line in sketch for point in line)
max_y = max(point[1] for line in sketch for point in line)

image = np.zeros((max_x, max_y, 3), dtype=np.uint8)
image = plot_lines(sketch, image)

compressed_sketch = exterminate_extra_points(sketch)

new_image = np.zeros((max_x, max_y, 3), dtype=np.uint8)
new_image = plot_lines(compressed_sketch, new_image)

while True:
    cv2.imshow('Original', image)
    cv2.imshow('Compressed', new_image)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cv2.imwrite("data/GoodDrawing.jpg", image)
cv2.imwrite("data/CompressedGoodDrawing.jpg", new_image)
json.dump(compressed_sketch, open('data/compressed_good_drawing.json', 'w'))
cv2.destroyAllWindows()
