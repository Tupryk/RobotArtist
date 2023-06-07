from loader import load_sketch, segment_line
from line_solver import line_solver
import numpy as np
from robotic import ry

ry.params_add({'physx/motorKp': 10000., 'physx/motorKd': 1000.})
ry.params_print()

C = ry.Config()
C.addFile(ry.raiPath('../rai-robotModels/scenarios/pandaSingle.g'))
C.view(False)

WHITE_BOARD_Z = 0.1
LIFTED_Z = WHITE_BOARD_Z - 0.05
DRAWING_MAX_WIDTH = 0.5
DRAWING_MAX_HEIGHT = 0.5

# Get drawing line data
sketch = load_sketch("output.json", max_dims=[DRAWING_MAX_WIDTH, DRAWING_MAX_HEIGHT])

### Create waypoints ###
waypoint_count = ( len(sketch) * 2 ) + sum([len(line) for line in sketch])

komos = []

for line in sketch:

    """
    # Starting waypoint (Above initial line point)
    way = C.addFrame('way'+str(way_point_counter)) \
        .setPosition([line[0][0], LIFTED_Z, line[0][1]]) \
        .setShape(ry.ST.marker, size=[.1])
    """
    for i in range(len(line)-1):
        komos.append(line_solver(line[i], line[i+1], C))

    """
    # End waypoint (Above ending line point)
    way = C.addFrame('way'+str(way_point_counter)) \
        .setPosition([line[-1][0], LIFTED_Z, line[-1][1]]) \
        .setShape(ry.ST.marker, size=[.1])
    """
    
    
C.view()

bot = ry.BotOp(C, False)
bot.home(C)

for k in komos:

    path = k.getPath()

    bot.move(path, [float(i) for i in range(waypoint_count)])
    while bot.getTimeToEnd() > 0:
        bot.sync(C, .1)

bot.home(C)
