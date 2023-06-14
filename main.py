from loader import load_sketch
from solvers import line_solver, pen_picker
import numpy as np
from robotic import ry

ry.params_add({'physx/motorKp': 10000., 'physx/motorKd': 1000.})
ry.params_print()

C = ry.Config()
C.addFile(ry.raiPath('../rai-robotModels/scenarios/pandaSingle.g'))

# Define pen to be grasped
C.addFrame("pen") \
    .setPosition([-0.25, .1, .675]) \
    .setShape(ry.ST.ssBox, size=[.02, .15, .02, .005]) \
    .setColor([1., .5, 0]) \
    .setMass(.1) \
    .setContact(True)

# TODO Get pen position (Right now we just know the simulated exact pen possition)

# Solve grasping
grasp_solver = pen_picker(C)

# Get drawing line data
sketch = load_sketch("square&line.json", max_dims=[0.3, 0.3], canvas_center=[0.25, 1.2])

LIFT_SPACE = 0.05
WHITE_BOARD_Z = 0.4

for j, line in enumerate(sketch):

    for i in range(len(line)-1):

        C.addFrame("Marker"+str(j*(len(sketch)+1))+str(i)) \
            .setPosition([line[i][0], WHITE_BOARD_Z, line[i][1]]) \
            .setShape(ry.ST.sphere, size=[.05, .005]) \
            .setColor([np.abs(line[i][0])*255, WHITE_BOARD_Z, np.abs(line[i][0])*255])

    C.addFrame("Marker"+str(j*(len(sketch)+1))+str(len(line)+1)) \
            .setPosition([line[-1][0], WHITE_BOARD_Z, line[-1][1]]) \
            .setShape(ry.ST.sphere, size=[.05, .005]) \
            .setColor([np.abs(line[i][0])*255, WHITE_BOARD_Z, np.abs(line[i][0])*255])


bot = ry.BotOp(C, False)
bot.home(C)

# Grasp pen
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

# Draw
last_point = None
for j, line in enumerate(sketch):

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
