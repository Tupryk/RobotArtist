import numpy as np
from robotic import ry
from modules.utils.line_math import line_length, give_lift_space

MAX_LIFT_SPACE = np.array([.15, .0, .0])
DRAW_SPEED = 0.1


def line_solver(waypoints, ry_config, debug=False):
    komo = ry.KOMO()
    komo.setConfig(ry_config, True)

    T = len(waypoints)
    komo.setTiming(T, 3, 1., 2)

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

    if debug:
        print("Solution: ", ret)
        komo.view_play(True)

    return komo.getPath()

def do_sketch(sketch, ry_config, bot):
    lift_space = MAX_LIFT_SPACE
    last_point = np.array(sketch[0][0])+lift_space
    for j, line in enumerate(sketch):

        print(f"Drawing line {j+1} of {len(sketch)}. ({(j*100/len(sketch)):.2f}% Done)")
    
        line_start = np.array(line[0])
        line_end = np.array(line[-1])

        # Move hand to line starting position
        lift_path = [last_point, line_start + lift_space, line_start]
        time_to_solve = line_length(lift_path)/(DRAW_SPEED*4)
        path = line_solver(lift_path[1:], ry_config, debug=False)

        bot.move(path, [time_to_solve])
        while bot.getTimeToEnd() > 0:
            bot.sync(ry_config, .1)

        # Draw single line
        bot.sync(ry_config, 0.)
        time_to_solve = line_length(line)/DRAW_SPEED
        path = line_solver(line[1:], ry_config, debug=False)

        bot.move(path, [time_to_solve])
        while bot.getTimeToEnd() > 0:
            bot.sync(ry_config, .1)

        # Lift hand from whiteboard
        bot.sync(ry_config, 0.)
        lift_path = [line_end, line_end + lift_space]
        time_to_solve = line_length(lift_path)/(DRAW_SPEED*4)
        path = line_solver(lift_path[1:], ry_config, debug=False)

        bot.move(path, [time_to_solve])
        while bot.getTimeToEnd() > 0:
            bot.sync(ry_config, .1)

        last_point = line_end + lift_space

        if j != len(sketch)-1:
            lift_space = np.array([give_lift_space(line_end, sketch[j+1][0]), 0, 0])