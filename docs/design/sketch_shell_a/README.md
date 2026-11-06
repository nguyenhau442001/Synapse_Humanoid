# Mini humanoid sketch — shell A (v0.5)

Claude Design canvas source for the 20-DoF mini humanoid. All geometry is drawn in mm
from the same dimensions as `sim/mini_humanoid/gen_robot.py` (303 mm tall, 481 g).

| File | Artboard |
| --- | --- |
| `Main.dc.html` | Front view, shell A |
| `Side.dc.html` | Side view with CoM (146 mm) |
| `Joints.dc.html` | 20-joint map, colored by servo (DS843MG / MG90S / SG90) |
| `canvas.json` | Canvas layout |

The `.dc.html` files render inside Claude Design (they load its `support.js`); the SVG
inside each file is plain and can be copied out on its own.
