from loader import load_sketch, segment_line
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
sketch = load_sketch("square&line.json", max_dims=[DRAWING_MAX_WIDTH, DRAWING_MAX_HEIGHT])

### Create waypoints ###
waypoint_count = ( len(sketch) * 2 ) + sum([len(line) for line in sketch])

way_point_counter = 0
for line in sketch:

    # Starting waypoint (Above initial line point)
    way = C.addFrame('way'+str(way_point_counter)) \
        .setPosition([line[0][0], LIFTED_Z, line[0][1]]) \
        .setShape(ry.ST.marker, size=[.1])
    
    way_point_counter += 1

    for point in line:
        way = C.addFrame('way'+str(way_point_counter)) \
            .setPosition([point[0], WHITE_BOARD_Z, point[1]]) \
            .setShape(ry.ST.marker, size=[.1])
        
        way_point_counter += 1

    # End waypoint (Above ending line point)
    way = C.addFrame('way'+str(way_point_counter)) \
        .setPosition([line[-1][0], LIFTED_Z, line[-1][1]]) \
        .setShape(ry.ST.marker, size=[.1])
    
    way_point_counter += 1

del way_point_counter
    
C.view()

# This should depend on line length
points_in_line = 20
phase_duration = 5.

points = segment_line(np.array([0, 0]), np.array([1, 1]), points_in_line)

komo = ry.KOMO()
komo.setConfig(C, True)
# Phase count
# Slices per phase
# Phase duration (in seconds)
# Degree of smothness
komo.setTiming(1, points_in_line, phase_duration, 2)
komo.addControlObjective([], 2, 1e-0)
# komo.addObjective([], ry.FS.accumulatedCollisions, [], ry.OT.eq);
# komo.addObjective([], ry.FS.jointLimits, [], ry.OT.ineq);

for i in range(20):
    komo.addObjective([float(i)], ry.FS.position, ['l_gripper'], ry.OT.eq, [1e1], points[i]);

ret = ry.NLP_Solver() \
    .setProblem(komo.nlp()) \
    .setOptions( stopTolerance=1e-2, verbose=0 ) \
    .solve()
print(ret)

komo.view(False, "Solution")

komo.view_close()
path = komo.getPath()

bot = ry.BotOp(C, False)
bot.home(C)

bot.move(path, [float(i+1) for i in range(waypoint_count)])
while bot.getTimeToEnd() > 0:
    bot.sync(C, .1)

bot.home(C)
