import json
import numpy as np


# Resolution is the distance between waypoints
def segment_line(point1, point2, point_between):

    points = []
    
    for i in range(point_between):
        new_point = point1 + (point2 - point1) * 0.5 * (1-np.cos(np.pi * i/(point_between-1)))
        points.append(new_point)

    return points


def load_sketch(file_path, max_dims=[0.5, 0.5], canvas_center=[0.25, 1]):

    original_sketch = json.load(open(file_path))
    scaled_sketch = []

    points = []
    for line in original_sketch:
        points += line
    x_coords = [point[0] for point in points]
    y_coords = [point[1] for point in points]
    min_x, max_x = min(x_coords), max(x_coords)
    min_y, max_y = min(y_coords), max(y_coords)

    for line in original_sketch:
        scaled_line = []

        scaling_factor_x = max_dims[0] / (max_x - min_x) if max_x != min_x else 1.
        scaling_factor_y = max_dims[1] / (max_y - min_y) if max_y != min_y else 1.

        for point in line:
            point[0] = (point[0] - min_x) * scaling_factor_x - (canvas_center[0] -(max_dims[1]*0.5))
            point[1] = (point[1] - min_y) * scaling_factor_y + (canvas_center[1] -(max_dims[1]*0.5))
            scaled_line.append(point)

        scaled_sketch.append(scaled_line)

    return scaled_sketch
