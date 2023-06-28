from robotic import ry

def checkWaypoints(C, waypoints):
    print(waypoints)
    for t in range(20):
        f = C.getFrame(f'Helper{t}')
        if t<len(waypoints):
            f.setPosition(waypoints[t])
        else:
            f.setPosition([0,0,0])

    #C.view(True, f'waypoint checker :#={len(waypoints)}')
        
def line_solver(waypoints, ry_config, debug=False):

    checkWaypoints(ry_config, waypoints)

    komo = ry.KOMO()
    komo.setConfig(ry_config, True)

    T = len(waypoints)
    komo.setTiming(T, 20, 1., 2)

    komo.addControlObjective([], 0, 1e-2)
    komo.addControlObjective([], 2, 1e1)

    komo.addObjective([], ry.FS.accumulatedCollisions, [], ry.OT.eq)
    komo.addObjective([], ry.FS.jointLimits, [], ry.OT.ineq)
    komo.addObjective([], ry.FS.vectorZ, ["l_gripper"], ry.OT.sos, [0.5], [1, 0, 0])
    komo.addObjective([T], ry.FS.qItself, [], ry.OT.eq, [1e1], [], 1)

    for i, point in enumerate(waypoints, start=0):
        komo.addObjective([i+1], ry.FS.position, ["pen_tip"], ry.OT.eq, [1e1], point)

    ret = ry.NLP_Solver() \
        .setProblem(komo.nlp()) \
        .setOptions(stopTolerance=1e-2, verbose=0) \
        .solve()
    
    print('line_solver return:', ret)
    if debug:
        komo.view_play(True)

    return komo.getPath()


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

    return komo.getPath()
