import cv2
import numpy as np
import matplotlib.pyplot as plt

DIMS = [800, 800]


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


def find_closest_point(points, target_point):
    points = np.array(points)
    target_point = np.array(target_point)

    distances = np.linalg.norm(points - target_point, axis=1)
    closest_point_index = np.argmin(distances)
    min_distance = distances[closest_point_index]

    return closest_point_index, min_distance


def joined_segments(segments, img=None, show=False):
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

            if show:
                height, width = img.shape
                lines_visual = np.zeros((height, width, 3), dtype=np.uint8)

                for line in final_lines:
                    random_color = [np.random.random()*255, np.random.random()*255, np.random.random()*255]
                    for i in range(len(line)-1):
                        cv2.line(lines_visual, (line[i][0], line[i][1]), (line[i+1][0], line[i+1][1]), (random_color[0], random_color[1], random_color[2]), 2)

                for i in range(len(new_line)-1):
                    cv2.line(lines_visual, (new_line[i][0], new_line[i][1]), (new_line[i+1][0], new_line[i+1][1]), (0, 255, 0), 2)

                cv2.imshow('drawing', lines_visual)

        final_lines.append(new_line)

        print("Average line length: ", np.round(sum([len(line) for line in final_lines])/len(final_lines), 2))
        print(f"Progress status: {((initial_seg_count-len(segments))*100/initial_seg_count):.2f}% done")

    return final_lines


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


def plot_lines(lines, image, show=False):
    for line in lines:
        random_color = [np.random.random()*255, np.random.random()*255, np.random.random()*255]
        for i in range(len(line)-1):
            cv2.line(image, (line[i][0], line[i][1]), (line[i+1][0], line[i+1][1]), (random_color[0], random_color[1], random_color[2]), 2)
    if show:
        plt.imshow(image)
        plt.show()
    return image


def get_drawing_bitmap(image, show=False):
    image = scale_image(image, DIMS)

    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    if show: cv2.imshow('Gray 1', gray)

    gray = cv2.GaussianBlur(gray, (7, 7), 0)
    if show: cv2.imshow('gaussian blur 2', gray)

    binary = cv2.adaptiveThreshold(gray, 255, cv2.ADAPTIVE_THRESH_MEAN_C, cv2.THRESH_BINARY, 11, 2)
    if show: cv2.imshow('addaptive threshold 3', binary)

    binary = cv2.bitwise_not(binary)
    nlabels, labels, stats, _ = cv2.connectedComponentsWithStats(binary, None, None, None, 8, cv2.CV_32S)
    areas = stats[1:,cv2.CC_STAT_AREA]
    result = np.zeros((labels.shape), np.uint8)
    for i in range(0, nlabels - 1):
        if areas[i] >= 100:   #keep
            result[labels == i + 1] = 255
    image = cv2.bitwise_not(result)
    if show: cv2.imshow('connectedComponentsWithStats 4', image)

    return image


# Prepare image
image = cv2.imread("../data/just_cutout.jpg")
image = get_drawing_bitmap(image, show=True)
    
# Apply probabilistic Hough transform
print("Calculating lines...")
minLineLength = 2
maxLineGap = 2

lines = cv2.HoughLinesP(cv2.bitwise_not(image), 1, np.pi/180, 5, minLineLength, maxLineGap)

lines = [line[0] for line in lines]
lines = [[point for point in line] for line in lines]

#image = plot_lines(lines, final)

sketch = joined_segments(lines)

canvas = np.zeros((DIMS[0], DIMS[1], 3), dtype=np.uint8)
image = plot_lines(sketch, canvas, show="Sketch lines")

compressed_sketch = exterminate_extra_points(sketch, threshold=15)

canvas = np.zeros((DIMS[0], DIMS[1], 3), dtype=np.uint8)
compressed_image = plot_lines(compressed_sketch, canvas, show="Compressed lines")

while True:
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break
