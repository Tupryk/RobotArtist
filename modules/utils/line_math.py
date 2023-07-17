import numpy as np
from robotic import ry


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
