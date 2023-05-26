import numpy as np
from robotic import ry

ry.params_add({'physx/motorKp': 10000., 'physx/motorKd': 1000.})
ry.params_print()

C = ry.Config()
C.addFile(ry.raiPath('../rai-robotModels/scenarios/pandaSingle.g'))
C.view(False)

WHITE_BOARD_Z = 0.1
LIFTED_Z = WHITE_BOARD_Z - 0.05
DRAWING_MAX_WIDTH = 0.7
DRAWING_MAX_HEIGHT = 0.7

# Get drawing line data
sketch = np.array([
    [ [-0.25, 1.0], [0.25, 1.0], [0.25, 0.675], [-0.25, 0.675], [-0.25, 1.0] ],
    [ [0.2, 0.8], [-0.2, 0.9] ]
])

# Scale the drawing


### Create waypoints ###
waypoint_count = ( len(sketch) * 2 ) + sum([len(line) for line in sketch])

way_point_counter = 0
for line in sketch:

    # Starting waypoint (Above initial line point)
    way = C.addFrame('way'+str(way_point_counter)) \
        .setPosition([line[0, 0], LIFTED_Z, line[0, 1]]) \
        .setShape(ry.ST.marker, size=[.1])
    
    way_point_counter += 1

    for point in line:
        way = C.addFrame('way'+str(way_point_counter)) \
            .setPosition([point[0], WHITE_BOARD_Z, point[1]]) \
            .setShape(ry.ST.marker, size=[.1])
        
        way_point_counter += 1

    # End waypoint (Above ending line point)
    way = C.addFrame('way'+str(way_point_counter)) \
        .setPosition([line[-1, 0], LIFTED_Z, line[-1, 1]]) \
        .setShape(ry.ST.marker, size=[.1])
    
    way_point_counter += 1

del way_point_counter
    
C.view()

komo = ry.KOMO()
komo.setConfig(C, True)
komo.setTiming(4., 1, 5., 0)
komo.addControlObjective([], 0, 1e-0)
komo.addObjective([], ry.FS.accumulatedCollisions, [], ry.OT.eq);
komo.addObjective([], ry.FS.jointLimits, [], ry.OT.ineq);

for i in range(waypoint_count):
    komo.addObjective([float(i+1)], ry.FS.poseDiff, ['l_gripper', 'way'+str(i)], ry.OT.eq, [1e1]);

ret = ry.NLP_Solver() \
    .setProblem(komo.nlp()) \
    .setOptions( stopTolerance=1e-2, verbose=0 ) \
    .solve()
print(ret)

komo.view(False, "waypoints solution")

komo.view_close()
path = komo.getPath()
print("Path: ")
print(path)

bot = ry.BotOp(C, False)
bot.home(C)

bot.move(path, [float(i+1) for i in range(waypoint_count)])
while bot.getTimeToEnd()>0:
    bot.sync(C, .1)

bot.home(C)

del bot
del C
