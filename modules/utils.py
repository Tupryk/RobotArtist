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

def give_lift_space(start_point, end_point, max_lift=.1, min_lift=.01):
    start_point = np.array(start_point)
    end_point = np.array(end_point)
    distance = np.linalg.norm(end_point-start_point)
    return min(max_lift, max(distance, min_lift))

def look_path(center=np.array([0, 0, 1.5]), radious=np.array([-.6, 0, 0]), count_=6, ry_config=None):
    points = []
    rotation_matrix = np.array([
        [np.cos(np.pi/count_), -np.sin(np.pi/count_), 0],
        [np.sin(np.pi/count_),  np.cos(np.pi/count_), 0],
        [                   0,                     0, 1],
    ])
    if ry_config:
        ry_config.addFrame("HeadCenter") \
            .setPosition(center) \
            .setShape(ry.ST.sphere, size=[.02]) \
            .setColor([0, 1, 0])
    radious = radious @ rotation_matrix
    radious = radious @ rotation_matrix
    for i in range(count_):
        radious = radious @ rotation_matrix

        if ry_config:
            ry_config.addFrame("Looking_point"+str(i)) \
                .setPosition(radious+center) \
                .setShape(ry.ST.sphere, size=[.02]) \
                .setColor([1, 0, 0])
        
        points.append(radious+center)
    return points
