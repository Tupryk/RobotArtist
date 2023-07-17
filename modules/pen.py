from robotic import ry


def pen_picker(ry_config):

    way0 = ry_config.addFrame("way0", "pen")
    way1 = ry_config.addFrame("way1", "pen")

    way0.setShape(ry.ST.marker, size=[.1])
    way0.setRelativePose("t(0 0 .1) d(90 0 0 1)")

    way1.setShape(ry.ST.marker, size=[.1])
    way1.setRelativePose("t(0 -.03 0) d(90 0 0 1)")

    komo = ry.KOMO()
    komo.setConfig(ry_config, True)
    komo.setTiming(2., 1, 5., 0)
    komo.addControlObjective([], 0, 1e-0)
    
    komo.addObjective([], ry.FS.accumulatedCollisions, [], ry.OT.eq)
    komo.addObjective([], ry.FS.jointLimits, [], ry.OT.ineq)

    komo.addObjective([], ry.FS.vectorY, ["l_gripper"], ry.OT.eq, [1e1], [0, 1, 0])
    komo.addObjective([], ry.FS.vectorZ, ["l_gripper"], ry.OT.eq, [1e1], [0, -1, 1])

    komo.addObjective([1.], ry.FS.poseDiff, ["l_gripper", "way0"], ry.OT.eq, [1e1])
    komo.addObjective([2.], ry.FS.poseDiff, ["l_gripper", "way1"], ry.OT.eq, [1e1])

    ry.NLP_Solver() \
        .setProblem(komo.nlp()) \
        .setOptions(stopTolerance=1e-2, verbose=0) \
        .solve()

    return komo.getPath()

def pickup_pen(ry_config, bot):

    path = pen_picker(ry_config)

    bot.gripperOpen(ry._left)
    while not bot.gripperDone(ry._left):
        bot.sync(ry_config, .1)

    bot.move(path, [2., 3.])
    while bot.getTimeToEnd() > 0:
        bot.sync(ry_config, .1)

    bot.gripperClose(ry._left)
    while not bot.gripperDone(ry._left):
        bot.sync(ry_config, .1)
