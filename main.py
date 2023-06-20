from modules.loader import load_sketch
from modules.solvers import line_solver, pen_picker
from modules.utils import sketch_plotter
import numpy as np
from robotic import ry


# TODO get WHITE_BOARD_Z from depth camera
LIFT_SPACE = 0.05
WHITE_BOARD_Z = 0.4

ry.params_add({'physx/motorKp': 10000., 'physx/motorKd': 1000.})
ry.params_print()

C = ry.Config()
C.addFile(ry.raiPath('../rai-robotModels/scenarios/pandaSingle.g'))

C.addFrame("pen") \
    .setPosition([-0.25, .1, .675]) \
    .setShape(ry.ST.ssBox, size=[.02, .15, .02, .005]) \
    .setColor([1., .5, 0]) \
    .setMass(.1) \
    .setContact(True)

sketch = load_sketch("data/output.json", max_dims=[0.3, 0.3], canvas_center=[0.25, 1.2])
C = sketch_plotter(sketch, C, WHITE_BOARD_Z)

bot = ry.BotOp(C, False)
bot.home(C)

# Grasp pen
grasp_solver = pen_picker(C)

grasp = False
if grasp:
    path = grasp_solver.getPath()

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
draw = True
if draw:
    last_point = None
    for j, line in enumerate(sketch):

        # Move back after finishing a line
        if last_point:
            komo = line_solver(np.array(last_point), np.array(line[0]), C, whiteboard_z=WHITE_BOARD_Z-LIFT_SPACE)

            path = komo.getPath()

            bot.move(path, [1.])
            while bot.getTimeToEnd() > 0:
                bot.sync(C, .1)

        for i in range(len(line)-1):

            komo = line_solver(np.array(line[i]), np.array(line[i+1]), C, whiteboard_z=WHITE_BOARD_Z)

            path = komo.getPath()

            bot.move(path, [1.])
            while bot.getTimeToEnd() > 0:
                bot.sync(C, .1)

        last_point = line[-1]

    bot.home(C)
