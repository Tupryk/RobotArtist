import numpy as np
from robotic import ry


def sketch_plotter(sketch, ry_config):

    sketch = np.array(sketch)
    max_x = max(sketch[:, :, 1].flatten())
    max_y = max(sketch[:, :, 2].flatten())
    min_x = min(sketch[:, :, 1].flatten())
    min_y = min(sketch[:, :, 2].flatten())

    total_points = 0
    for line in sketch:
        for point in line:

            ry_config.addFrame("Marker"+str(total_points)) \
                .setPosition(point) \
                .setShape(ry.ST.sphere, size=[.005]) \
                .setColor([255*(point[0]-min_x)/(max_x-min_x), 255*(point[1]-min_y)/(max_y-min_y), 0])
            
            total_points += 1

    return ry_config
