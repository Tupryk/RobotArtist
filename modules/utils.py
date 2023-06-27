import numpy as np
from robotic import ry


def sketch_plotter(sketch, ry_config):

    max_x = max(point[1] for line in sketch for point in line)
    max_y = max(point[2] for line in sketch for point in line)
    min_x = min(point[1] for line in sketch for point in line)
    min_y = min(point[2] for line in sketch for point in line)

    total_points = 0
    for line in sketch:
        for point in line:

            ry_config.addFrame("Marker"+str(total_points)) \
                .setPosition(point) \
                .setShape(ry.ST.sphere, size=[.005]) \
                .setColor([(point[1]-min_x)/(max_x-min_x), (point[2]-min_y)/(max_y-min_y), 0])
            
            total_points += 1

    return ry_config

def line_length(line):
    sum_ = 0
    for i in range(len(line)-1):
        sum_ += np.sqrt(np.power(line[i][0]-line[i+1][0], 2)+np.power(line[i][1]-line[i+1][1], 2)+np.power(line[i][2]-line[i+1][2], 2))
    return sum_
