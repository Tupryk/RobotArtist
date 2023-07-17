from robotic import ry

from config import generate_config
from modules.utils.loader import load_sketch
from modules.utils.line_math import sketch_plotter

from modules.face_capture import search_faces
from modules.sketcher import do_sketch
from modules.pen import pickup_pen

SEARCH_FACES = True
PICKUP_PEN = True
DO_SKETCH = True

CANVAS_CENTER = [-.05, 1.5, -.4]
SCKETCH_DIMS = [.3, .3]


if __name__ == "__main__":

    C = generate_config(CANVAS_CENTER, SCKETCH_DIMS)

    bot = ry.BotOp(C, False)
    bot.home(C)

    # Find Faces
    if SEARCH_FACES:
        sketch_2d = search_faces(C, bot)
    # Load predefined sketch
    else: sketch = load_sketch("data/output.json", CANVAS_CENTER, SCKETCH_DIMS, invert_y=True)

    # Display sketch in simulation
    C = sketch_plotter(sketch, C)

    # Grasp pen
    if PICKUP_PEN: pickup_pen(C, bot)

    bot.home(C)
    bot.sync(C, .1)

    # Draw sketch
    if DO_SKETCH: do_sketch(sketch, C, bot)

    bot.home(C)
