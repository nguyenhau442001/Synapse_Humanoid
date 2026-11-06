# Mini humanoid sketch — shell A (v0.5)

Claude Design canvas source for the 20-DoF mini humanoid. All geometry is drawn in mm
from the same dimensions as `sim/mini_humanoid/gen_robot.py` (303 mm tall, 481 g).

| Front | Side |
| --- | --- |
| ![Front view](images/main.png) | ![Side view](images/side.png) |

![20-joint servo map](images/joints.png)

![BOM](images/bom.png)

| File | Artboard |
| --- | --- |
| `Main.dc.html` | Front view, shell A, with part sizes and joint heights |
| `Side.dc.html` | Side view with depths, internal layout and CoM (146 mm) |
| `Joints.dc.html` | 20-joint map, colored by servo (DS843MG / MG90S / SG90) |
| `BOM.dc.html` | Bill of materials, about 6.68M VND |
| `canvas.json` | Canvas layout |
| `images/*.png` | Rendered exports of the three artboards |

The `.dc.html` files render inside Claude Design (they load its `support.js`); the SVG
inside each file is plain and can be copied out on its own.
