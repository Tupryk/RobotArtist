from robotic import ry


def line_solver(waypoints, ry_config, debug=False):

    komo = ry.KOMO()
    #ry.getFrame().getPosition()
    komo.setConfig(ry_config, True)

    komo.setTiming(len(waypoints)+2, 3, 10, 2)

    komo.addControlObjective([], 0, 1e-2)
    #komo.addControlObjective([], 1, 1)
    komo.addControlObjective([], 2, 1e2)

    komo.addObjective([], ry.FS.accumulatedCollisions, [], ry.OT.eq)
    komo.addObjective([], ry.FS.jointLimits, [], ry.OT.ineq)

    komo.addObjective([], ry.FS.vectorZ, ["l_gripper"], ry.OT.sos, [1e0], [1, 0, 0])

    for i, point in enumerate(waypoints):
        komo.addObjective([i+2], ry.FS.position, ['l_gripper'], ry.OT.eq, [1e0], point)

    ret = ry.NLP_Solver() \
        .setProblem(komo.nlp()) \
        .setOptions(stopTolerance=1e-2, verbose=0) \
        .solve()
    
    print(ret)
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
