from robotic import ry


def generate_config(CANVAS_CENTER, SCKETCH_DIMS, ON_REAL):

    ry.params_file('rai.cfg')

    C = ry.Config()
    C.addFile(ry.raiPath(f'../rai-robotModels/scenarios/pandaSingle{"_jenga" if ON_REAL else ""}.g'))

    C.addFrame("pen") \
        .setPosition([.0, .3, .675]) \
        .setShape(ry.ST.ssBox, size=[.02, .1, .02, .005]) \
        .setColor([1., .5, 0]) \
        .setMass(.1) \
        .setContact(True)

    C.addFrame("whiteboard") \
        .setShape(ry.ST.ssBox, size=[.01, 2.4, 1.2, .005]) \
        .setPosition([CANVAS_CENTER[2], CANVAS_CENTER[0]+SCKETCH_DIMS[0]+.1, CANVAS_CENTER[1]+SCKETCH_DIMS[1]]) \
        .setColor([.3, .3, 1., 0.5]) \
        .setContact(True)

    pen_tip = C.addFrame("pen_tip", "l_gripper")
    pen_tip.setRelativePose("t(.0 .09 .0)")
    pen_tip.setShape(ry.ST.sphere, size=[.005])
    pen_tip.setColor([1., .0, 1.])

    f = C.addFrame("l_gripperCamera", "l_gripper")
    f.setShape(ry.ST.camera, [.1])
    f.addAttributes({'focalLength':0.895, 'width':640., 'height':360.})

    return C
