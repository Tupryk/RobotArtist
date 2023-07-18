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

def find_closest_point(points, target_point):
    points = np.array(points)
    target_point = np.array(target_point)

    distances = np.linalg.norm(points - target_point, axis=1)
    closest_point_index = np.argmin(distances)
    min_distance = distances[closest_point_index]

    return closest_point_index, min_distance


def joined_segments(segments):
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
