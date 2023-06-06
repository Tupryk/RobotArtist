import numpy as np
from robotic import ry
from loader import segment_line

def line_solver(point1, point2, ry_config, resolution=0.01, duration_per_meter=10.):

    vector = point2 - point1
    length = np.linalg.norm(vector)
    
    points_in_line = int(np.floor(length/resolution))
    phase_duration = length/duration_per_meter

    points = segment_line(point1, point2, points_in_line)

    komo = ry.KOMO()
    komo.setConfig(ry_config, True)

    komo.setTiming(1, points_in_line, phase_duration, 2)
    komo.addControlObjective([], 2, 1e-0)
    # komo.addObjective([], ry.FS.accumulatedCollisions, [], ry.OT.eq);
    # komo.addObjective([], ry.FS.jointLimits, [], ry.OT.ineq);

    for i in range(points_in_line):
        komo.addObjective([float(i)], ry.FS.position, ['l_gripper'], ry.OT.eq, [1e1], points[i]);

    ret = ry.NLP_Solver() \
        .setProblem(komo.nlp()) \
        .setOptions( stopTolerance=1e-2, verbose=0 ) \
        .solve()
