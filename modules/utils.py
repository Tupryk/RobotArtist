import numpy as np
from robotic import ry


def sketch_plotter(sketch, ry_config, whiteboard_z=.0):
    total_points = 0
    for line in sketch:
        for point in line:

            ry_config.addFrame("Marker"+str(total_points)) \
                .setPosition([point[0], whiteboard_z, point[1]]) \
                .setShape(ry.ST.sphere, size=[.05, .005]) \
                .setColor([np.abs(point[0])*255, whiteboard_z, np.abs(point[1])*255])
            
            total_points += 1

    return ry_config
