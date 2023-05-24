import sys, os
#sys.path.append(os.path.expanduser('~/git/rai-python/build'))
#import libry as ry
from robotic import ry


C = ry.Config()
C.addFile('mini.g')
C.view()

C.watchFile('mini.g')

q = C.getJointState()
print(q)

q[0] = q[0] + .5
C.setJointState(q)
C.view()

frameC = C.frame('C')
print('pos:', frameC.getPosition(), 'quat:', frameC.getQuaternion())

q[0] = q[0] + .5
C.setJointState(q)
print('pos:', frameC.getPosition(), 'quat:', frameC.getQuaternion())

[y,J] = C.eval(ry.FS.position, ['C'])
print('position of C:', y, '\nJacobian:', J)
type(J)
