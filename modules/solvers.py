import numpy as np
from robotic import ry
from modules.utils import segment_line


def line_solver(point1, point2, ry_config, resolution=0.01, speed=0.01, whiteboard_z=0.5):
    
    vector = point2 - point1
    length = np.linalg.norm(vector)
    
    points_in_line = int(length/resolution)
    phase_duration = length/speed

    points = segment_line(point1, point2, points_in_line)

    for i, _ in enumerate(points):
        points[i] = [points[i][0], whiteboard_z, points[i][1]]

    komo = ry.KOMO()
    komo.setConfig(ry_config, True)

    komo.setTiming(1, len(points), phase_duration, 2)
    komo.addControlObjective([], 2, 1)
    komo.addObjective([], ry.FS.accumulatedCollisions, [], ry.OT.eq)
    komo.addObjective([], ry.FS.jointLimits, [], ry.OT.ineq)
    komo.addObjective([], ry.FS.vectorZ, ["l_gripper"], ry.OT.eq, [1e1], [0, -1, 0])
    komo.addObjective([], ry.FS.vectorX, ["l_gripper"], ry.OT.eq, [1e1], [1, 0, 0])

    for i, point in enumerate(points):
        komo.addObjective([float(i+1)/len(points)], ry.FS.position, ['l_gripper'], ry.OT.eq, [1e1], point)

    ret = ry.NLP_Solver() \
        .setProblem(komo.nlp()) \
        .setOptions(stopTolerance=1e-2, verbose=0) \
        .solve()
    
    print(ret)
    # komo.view_play()
    return komo

def pen_picker(ry_config):
    # TODO Get pen position (Right now we just know the simulated exact pen possition)

    way0 = ry_config.addFrame("way0", "pen")
    way1 = ry_config.addFrame("way1", "pen")

    way0.setShape(ry.ST.marker, size=[.1])
    way0.setRelativePose("t(0 0 .1) d(90 0 0 1)")

    way1.setShape(ry.ST.marker, size=[.1])
    way1.setRelativePose("d(90 0 0 1)")

    komo = ry.KOMO()
    komo.setConfig(ry_config, True)
    komo.setTiming(2., 1, 5., 0)
    komo.addControlObjective([], 0, 1e-0)
    komo.addObjective([], ry.FS.vectorZ, ["l_gripper"], ry.OT.eq, [1e1], [0, 0, -1])
    # komo.addObjective([], ry.FS.accumulatedCollisions, [], ry.OT.eq);
    # komo.addObjective([], ry.FS.jointLimits, [], ry.OT.ineq);

    komo.addObjective([1.], ry.FS.poseDiff, ["l_gripper", "way0"], ry.OT.eq, [1e1])
    komo.addObjective([2.], ry.FS.poseDiff, ["l_gripper", "way1"], ry.OT.eq, [1e1])

    ret = ry.NLP_Solver() \
        .setProblem(komo.nlp()) \
        .setOptions(stopTolerance=1e-2, verbose=0) \
        .solve()
    
    print(ret)

    return komo
