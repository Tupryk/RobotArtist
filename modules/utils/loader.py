import json


def sketch_to_3d(original_sketch, CANVAS_CENTER, SCKETCH_DIMS):

    # Get the maximum possible x and y coordinates
    x_coords = [point[0] for points in original_sketch for point in points]
    y_coords = [point[1] for points in original_sketch for point in points]
    min_x, max_x = min(x_coords), max(x_coords)
    min_y, max_y = min(y_coords), max(y_coords)

    # Get the scaling factors possible x and y coordinates
    scaling_factor_x = SCKETCH_DIMS[0] / (max_x - min_x) if max_x != min_x else 1.
    scaling_factor_y = SCKETCH_DIMS[1] / (max_y - min_y) if max_y != min_y else 1.

    scaled_sketch = []
    for line in original_sketch:
        scaled_line = []

        for point in line:
            point[0] = (point[0] - min_x) * scaling_factor_x + CANVAS_CENTER[0]
            point[1] = (point[1] - min_y) * scaling_factor_y + CANVAS_CENTER[1]
            
            scaled_line.append([CANVAS_CENTER[2], point[0], point[1]])

        scaled_sketch.append(scaled_line)

    return scaled_sketch

    
def load_sketch(file_path, CANVAS_CENTER, SCKETCH_DIMS, invert_y=False):
    original_sketch = json.load(open(file_path))
    if invert_y:
        for i, line in enumerate(original_sketch):
            for j, _ in enumerate(line):
                original_sketch[i][j][1] *= -1
    return sketch_to_3d(original_sketch, CANVAS_CENTER, SCKETCH_DIMS)
    
