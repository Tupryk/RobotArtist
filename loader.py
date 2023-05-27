import json


def load_sketch(file_path, max_dims=[0.5, 0.5], whiteboard_center=[0.25, 1]):

    original_sketch = json.load(open(file_path))
    scaled_sketch = []

    for line in original_sketch:
        scaled_line = []
        for point in line:
            point[0] = (max_dims[0]*point[0]) - (max_dims[0]*0.5)
            point[1] = (max_dims[1]*point[1]) - (max_dims[1]*0.5)
            point[0] += whiteboard_center[0] - (max_dims[0]*0.5)
            point[1] += whiteboard_center[1] - (max_dims[1]*0.5)
            scaled_line.append(point)
        scaled_sketch.append(scaled_line)

    return scaled_sketch
