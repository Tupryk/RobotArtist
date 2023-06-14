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

line_solvers = []
for line in sketch:
    for i in range(len(line)-1):
        line_solvers.append(line_solver(np.array(line[i]), np.array(line[i+1]), C, whiteboard_z=0.3))


bot = ry.BotOp(C, False)
bot.home(C)

# Grasp pen
"""
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
"""

bot.home(C)

# Draw
for k in line_solvers:

    path = k.getPath()

    bot.move(path, [1.])
    while bot.getTimeToEnd() > 0:
        bot.sync(C, .1)

bot.home(C)
