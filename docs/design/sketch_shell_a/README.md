# Mini humanoid sketches — shell A hero, shell B villain (v0.5)

Claude Design canvas source for the 20-DoF mini humanoid. Shell A (cute) is the hero,
shell B (armor) is the villain; both share the same skeleton and joint axes. All geometry is drawn in mm
from the same dimensions as `sim/mini_humanoid/gen_robot.py` (303 mm tall, 481 g).

| Front | Side |
| --- | --- |
| ![Front view](images/main.png) | ![Side view](images/side.png) |

### Villain · shell B

![Shell B villain](images/shell_b.png)

Pauldrons widen it to about 132 mm (A: 119) and must ride on the upper arm so shoulder roll stays free.

![20-joint servo map](images/joints.png)

![BOM](images/bom.png)

![GA drawing](images/drawing.png)

| File | Artboard |
| --- | --- |
| `Main.dc.html` | Shell A (hero) front view, with part sizes and joint heights |
| `ShellB.dc.html` | Shell B (villain) armor, front and side |
| `Side.dc.html` | Side view with depths, internal layout and CoM (146 mm) |
| `Joints.dc.html` | 20-joint map, colored by servo (DS843MG / MG90S / SG90) |
| `BOM.dc.html` | Bill of materials, about 6.68M VND |
| `Drawing.dc.html` | GA drawing, first-angle projection (front, side, top), A1 at 1:1 |
| `canvas.json` | Canvas layout |
| `images/*.png` | Rendered exports of the three artboards |

The `.dc.html` files render inside Claude Design (they load its `support.js`); the SVG
inside each file is plain and can be copied out on its own.
