from modules.loader import load_sketch
from modules.solvers import line_solver, pen_picker
from modules.utils import sketch_plotter, line_length, give_lift_space, look_path
import numpy as np
from robotic import ry

# TODO get WHITE_BOARD_Z from depth camera
MAX_LIFT_SPACE = np.array([.15, .0, .0])
DRAW_SPEED = 0.1
WHITEBOARD_DEPTH = .4
SCKETCH_DIMS = [.3, .3]
CANVAS_CENTER = [-.05, 1.5]

ry.params_file('rai.cfg')
ry.params_print()

C = ry.Config()
C.addFile(ry.raiPath('../rai-robotModels/scenarios/pandaSingle.g'))

C.addFrame("pen") \
    .setPosition([.0, .3, .675]) \
    .setShape(ry.ST.ssBox, size=[.02, .1, .02, .005]) \
    .setColor([1., .5, 0]) \
    .setMass(.1) \
    .setContact(True)

C.addFrame("whiteboard") \
    .setPosition([-WHITEBOARD_DEPTH, CANVAS_CENTER[0]+SCKETCH_DIMS[0]+.1, CANVAS_CENTER[1]-SCKETCH_DIMS[1]]) \
    .setShape(ry.ST.ssBox, size=[.01, SCKETCH_DIMS[0]*2, SCKETCH_DIMS[1]*2, .005]) \
    .setColor([.3, .3, 1., 0.5]) \
    .setContact(True)

pen_tip = C.addFrame("pen_tip", "l_gripper")
pen_tip.setRelativePose("t(.0 .05 -.05)")
pen_tip.setShape(ry.ST.sphere, size=[.005])
pen_tip.setColor([1., .0, 1.])

bot = ry.BotOp(C, False)
bot.home(C)

# Find Faces
find_faces = True
if find_faces:
    points = look_path(ry_config=C)
    index = 0
    while True:
        komo = ry.KOMO()
        komo.setConfig(C, True)
        komo.setTiming(1, 3, 1., 2)
        komo.addControlObjective([], 0, 1e-2)
        komo.addControlObjective([], 2, 1e1)

        komo.addObjective([], ry.FS.accumulatedCollisions, [], ry.OT.eq)
        komo.addObjective([], ry.FS.jointLimits, [], ry.OT.ineq)

        komo.addObjective([], ry.FS.vectorZ, ["l_gripper"], ry.OT.sos, [2.], [-1, 0, 0])
        komo.addObjective([], ry.FS.vectorY, ["l_gripper"], ry.OT.eq, [0.5], [0, 0, 1])

        komo.addObjective([1.], ry.FS.position, ["pen_tip"], ry.OT.eq, [1e1], points[index])

        ret = ry.NLP_Solver() \
            .setProblem(komo.nlp()) \
            .setOptions(stopTolerance=1e-2, verbose=0) \
            .solve()
        print(ret)
        
        bot.move(komo.getPath(), [1.])
        while bot.getTimeToEnd() > 0:
            bot.sync(C, .1)

        index += 1
        if index >= len(points): index = 0

        # Take picture
        # if face: break

    # Make face into sketch
else:
    sketch = load_sketch("data/compressed_good_drawing.json", max_dims=SCKETCH_DIMS, canvas_center=CANVAS_CENTER, whiteboard_depth=WHITEBOARD_DEPTH)

C = sketch_plotter(sketch, C)

# Grasp pen
grasp = True
if grasp:
    path = pen_picker(C)

    bot.gripperOpen(ry._left)
    while not bot.gripperDone(ry._left):
        bot.sync(C, .1)

    bot.move(path, [2., 3.])
    while bot.getTimeToEnd() > 0:
        bot.sync(C, .1)

    bot.gripperClose(ry._left)
    while not bot.gripperDone(ry._left):
        bot.sync(C, .1)

    bot.home(C)

# Draw sketch
bot.sync(C, .1)
draw = True
if draw:
    lift_space = MAX_LIFT_SPACE
    last_point = np.array(sketch[0][0])+lift_space
    for j, line in enumerate(sketch):

        print(f"Drawing line {j+1} of {len(sketch)}. ({(j*100/len(sketch)):.2f}% Done)")
    
        line_start = np.array(line[0])
        line_end = np.array(line[-1])

        # Move hand to line starting position
        lift_path = [last_point, line_start + lift_space, line_start]
        time_to_solve = line_length(lift_path)/(DRAW_SPEED*4)
        path = line_solver(lift_path[1:], C, debug=False)

        bot.move(path, [time_to_solve])
        while bot.getTimeToEnd() > 0:
            bot.sync(C, .1)

        # Draw single line
        try:
            bot.sync(C, 0.)
            time_to_solve = line_length(line)/DRAW_SPEED
            path = line_solver(line[1:], C, debug=False)

            bot.move(path, [time_to_solve])
            while bot.getTimeToEnd() > 0:
                bot.sync(C, .1)
        except: pass

        # Lift hand from whiteboard
        bot.sync(C, 0.)
        lift_path = [line_end, line_end + lift_space]
        time_to_solve = line_length(lift_path)/(DRAW_SPEED*4)
        path = line_solver(lift_path[1:], C, debug=False)

        bot.move(path, [time_to_solve])
        while bot.getTimeToEnd() > 0:
            bot.sync(C, .1)

        last_point = line_end + lift_space

        if j != len(sketch)-1:
            lift_space = np.array([give_lift_space(line_end, sketch[j+1][0]), 0, 0])
