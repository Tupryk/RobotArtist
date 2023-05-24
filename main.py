#!/usr/bin/env python
# coding: utf-8

# In[1]:


import sys, os
#sys.path.append(os.path.expanduser('~/git/rai-python/build'))
#import libry as ry
from robotic import ry
import numpy as np
import time


ry.params_add({'physx/motorKp': 10000., 'physx/motorKd': 1000.})
ry.params_print()

C = ry.Config()
C.addFile(ry.raiPath('../rai-robotModels/scenarios/pandaSingle.g'))
C.view(False)

C.addFrame('box')     .setPosition([-.25,.1,.675])     .setShape(ry.ST.ssBox, size=[.05,.05,.05,.005])     .setColor([1,.5,0])     .setMass(.1)     .setContact(True)
C.view()

way0 = C.addFrame('way0', 'box')
way1 = C.addFrame('way1', 'box')

way0.setShape(ry.ST.marker, size=[.1])
way0.setRelativePose('t(0 0 .1) d(90 0 0 1)')

way1.setShape(ry.ST.marker, size=[.1])
way1.setRelativePose('d(90 0 0 1)')

C.view()

# define a 2 waypoint problem in KOMO
komo = ry.KOMO()
komo.setConfig(C, True)
komo.setTiming(2., 1, 5., 0)
komo.addControlObjective([], 0, 1e-0)
komo.addObjective([], ry.FS.accumulatedCollisions, [], ry.OT.eq);
komo.addObjective([], ry.FS.jointLimits, [], ry.OT.ineq);
komo.addObjective([1.], ry.FS.poseDiff, ['l_gripper', 'way0'], ry.OT.eq, [1e1]);
komo.addObjective([2.], ry.FS.poseDiff, ['l_gripper', 'way1'], ry.OT.eq, [1e1]);

ret = ry.NLP_Solver()     .setProblem(komo.nlp())     .setOptions( stopTolerance=1e-2, verbose=4 )     .solve()
print(ret)

komo.view(False, "waypoints solution")

komo.view_close()
path = komo.getPath()

bot = ry.BotOp(C, False)
bot.home(C)

bot.gripperOpen(ry._left)
while not bot.gripperDone(ry._left):
    bot.sync(C, .1)

bot.move(path, [2., 3.])
while bot.getTimeToEnd()>0:
    bot.sync(C, .1)

bot.gripperClose(ry._left)
while not bot.gripperDone(ry._left):
    bot.sync(C, .1)

bot.home(C)

bot.gripperOpen(ry._left)
while not bot.gripperDone(ry._left):
    bot.sync(C, .1)

del bot
del C
