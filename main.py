from modules.loader import load_sketch
from modules.solvers import line_solver, pen_picker
from modules.utils import sketch_plotter, line_length
import numpy as np
from robotic import ry
import time

# TODO get WHITE_BOARD_Z from depth camera
LIFT_SPACE = np.array([.15, .0, .0])
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

sketch = load_sketch("data/output.json", max_dims=SCKETCH_DIMS, canvas_center=[-.05, 0], whiteboard_depth=WHITEBOARD_DEPTH)
C = sketch_plotter(sketch, C)

bot = ry.BotOp(C, False)
bot.home(C)

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
    last_point = np.array(sketch[0][0])+LIFT_SPACE
    for j, line in enumerate(sketch):
    
        line_start = np.array(line[0])
        line_end = np.array(line[-1])

        # Move hand to line starting position
        lift_path = [last_point, line_start + LIFT_SPACE, line_start]
        time_to_solve = line_length(lift_path)/DRAW_SPEED
        path = line_solver(lift_path[1:], C, debug=False)

        bot.move(path, [time_to_solve])
        while bot.getTimeToEnd() > 0:
            bot.sync(C, .1)

        # Draw single line
        bot.sync(C, 0.)
        time_to_solve = line_length(line)/DRAW_SPEED
        path = line_solver(line[1:], C, debug=False)


        bot.move(path, [time_to_solve])
        while bot.getTimeToEnd() > 0:
            bot.sync(C, .1)

        # Lift hand from whiteboard
        bot.sync(C, 0.)
        lift_path = [line_end, line_end + LIFT_SPACE]
        time_to_solve = line_length(lift_path)/DRAW_SPEED
        path = line_solver(lift_path[1:], C, debug=False)

        bot.move(path, [time_to_solve])
        while bot.getTimeToEnd() > 0:
            bot.sync(C, .1)

        last_point = line_end + LIFT_SPACE