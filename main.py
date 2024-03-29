from robotic import ry

from config import generate_config
from modules.utils.loader import load_sketch, sketch_to_3d
from modules.utils.visual import sketch_plotter

from modules.face_capture import search_faces
from modules.sketcher import do_sketch
from modules.pen import pickup_pen

SEARCH_FACES = False
PICKUP_PEN = False
DO_SKETCH = True
SIGN = True

CANVAS_CENTER = [-.05, 1.2, .6]
SCKETCH_DIMS = [.3, .3]

ON_REAL = False


signature = load_sketch("data/signature.json", [CANVAS_CENTER[0]-SCKETCH_DIMS[0]*.2, CANVAS_CENTER[1]-SCKETCH_DIMS[1]*.2, CANVAS_CENTER[2]], [.08, .02], invert_y=False, invert_x=True)

if __name__ == "__main__":

    C = generate_config(CANVAS_CENTER, SCKETCH_DIMS, ON_REAL)

    bot = ry.BotOp(C, ON_REAL)
    bot.home(C)

    # Find Faces
    if SEARCH_FACES:
        sketch_2d = search_faces(C, bot, simple=True, show=True)
        sketch = sketch_to_3d(sketch_2d, CANVAS_CENTER, SCKETCH_DIMS)
    # Load predefined sketch
    else: sketch = load_sketch("data/output.json", CANVAS_CENTER, SCKETCH_DIMS, invert_y=True)

    bot.home(C)
    bot.sync(C, .1)

    # Display sketch in simulation
    C = sketch_plotter(sketch, C)
    C = sketch_plotter(signature, C)

    # Grasp pen
    if PICKUP_PEN: pickup_pen(C, bot)

    bot.home(C)
    bot.sync(C, .1)

    # Draw sketch
    if DO_SKETCH: do_sketch(sketch, C, bot)

    if SIGN:
        do_sketch(signature, C, bot)

    bot.home(C)
