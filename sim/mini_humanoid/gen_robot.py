"""Generate MJCF (MuJoCo) and URDF (Foxglove / ROS 2) for the 20-DoF mini humanoid (v0.5).

Single source of truth: every size, mass and joint below comes from the design doc
"Mini Humanoid - Thiết kế phần cứng". Edit here, re-run, both files update together.

    python gen_robot.py            -> humanoid.xml, humanoid.urdf

Units in this file: millimetres and grams. Output files use metres and kilograms.
Frame: x forward, y left, z up. Feet soles touch z = 0 in the default pose.
"""
import os
from dataclasses import dataclass, field
from math import pi
from pathlib import Path

# ---------------------------------------------------------------- servos
# Stall torque (N.m) is the hard limit given to the simulator.
# Firmware should keep continuous load below ~1/3 of stall.
SERVOS = {
    # heavy joints: hip roll/pitch, knee, ankle pitch/roll
    "DS843MG": dict(stall=0.471, speed=0.10, mass=8.5, size=(23, 9, 23)),     # Corona, 4.8 kg.cm @6V, metal gear
    # medium: hip yaw, shoulder pitch (metal gears survive falls)
    "MG90S": dict(stall=0.216, speed=0.08, mass=13.4, size=(22.8, 12.2, 28.5)),  # 2.2 kg.cm
    # light: shoulder roll, elbow, head
    "SG90": dict(stall=0.177, speed=0.10, mass=9.0, size=(23, 12.2, 29)),      # 1.8 kg.cm @4.8V, plastic gear
    # previous leg servo, kept for comparison runs
    "MG92B": dict(stall=0.343, speed=0.08, mass=14.0, size=(22.8, 12, 31)),   # 3.5 kg.cm @6V
}
LEG_HEAVY = "DS843MG"
# Optional cheaper mix: set VARIANT=mix to keep MG92B on the less-loaded leg joints.
LEG_SERVO = {}
if os.environ.get("VARIANT") == "mix":
    LEG_SERVO = {"hip_pitch": "MG92B", "ankle_pitch": "MG92B", "ankle_roll": "MG92B"}
KP = 8.0          # position gain, N.m/rad (stiffness of a hobby servo, tune vs. real)
DAMPING = 0.02    # joint damping, N.m.s/rad
ARMATURE = 5e-4   # reflected rotor inertia through the gearbox, kg.m^2

# ---------------------------------------------------------------- colours
C = {
    "frame": (0.88, 0.88, 0.86, 1),
    "shell": (0.95, 0.95, 0.93, 1),
    "mg92b": (0.36, 0.56, 0.90, 1),
    "ds843": (0.36, 0.56, 0.90, 1),
    "sg90": (0.45, 0.70, 0.95, 1),
    "mg90s": (0.30, 0.36, 0.52, 1),
    "battery": (0.93, 0.55, 0.42, 1),
    "pcb": (0.36, 0.75, 0.60, 1),
    "lens": (0.10, 0.10, 0.12, 1),
    "hidden": (0, 0, 0, 0),
}


@dataclass
class Geom:
    name: str
    size: tuple          # full box size x, y, z (mm)
    pos: tuple           # centre, relative to body frame (mm)
    mass: float          # g
    color: str


@dataclass
class Joint:
    name: str
    axis: tuple
    range_deg: tuple
    servo: str


@dataclass
class Body:
    name: str
    pos: tuple                       # relative to parent body (mm)
    joint: Joint | None
    geoms: list
    children: list = field(default_factory=list)


HIP_Z = 115      # hip point H: hip yaw, roll and pitch axes all pass through it (mm above floor)
THIGH = SHIN = 45
ANKLE_Z = 25     # ankle point A: ankle pitch and roll axes intersect here


def sv(name, kind, pos, orient="x"):
    """Servo body as a box geom with the real mass and size of that servo type.
    orient: which axis the servo's long side (L) runs along."""
    L, W, H = SERVOS[kind]["size"]
    size = {"x": (L, W, H), "y": (W, L, H), "z": (W, H, L)}[orient]
    color = {"DS843MG": "ds843", "MG90S": "mg90s", "SG90": "sg90", "MG92B": "mg92b"}[kind]
    return Geom(name, size, pos, SERVOS[kind]["mass"], color)


def leg(side):
    """6 DoF: hip yaw -> hip roll -> hip pitch meet at H; ankle pitch -> ankle roll meet at A."""
    s = 1 if side == "l" else -1
    hv = LEG_HEAVY

    def J(j):
        return LEG_SERVO.get(j, hv)

    return Body(f"{side}_hip_yaw", (0, 22 * s, 0),
        Joint(f"{side}_hip_yaw", (0, 0, 1), (-45, 45), "MG90S"),
        [sv(f"{side}_hip_yaw_servo", "MG90S", (0, 0, 22), "z")],                  # upright in the pelvis
        [Body(f"{side}_hip_roll", (0, 0, 0),
            Joint(f"{side}_hip_roll", (1, 0, 0), (-25, 25), J("hip_roll")),
            [sv(f"{side}_hip_roll_servo", J("hip_roll"), (-14, 0, 0), "y")],  # behind H, shaft on x axis
            [Body(f"{side}_hip_pitch", (0, 0, 0),
                Joint(f"{side}_hip_pitch", (0, 1, 0), (-90, 45), J("hip_pitch")),
                [sv(f"{side}_hip_pitch_servo", J("hip_pitch"), (0, 0, -9), "x"),
                 Geom(f"{side}_thigh", (18, 18, THIGH), (0, 0, -THIGH / 2), 11, "shell")],
                [Body(f"{side}_knee", (0, 0, -THIGH),
                    Joint(f"{side}_knee", (0, 1, 0), (0, 120), J("knee")),
                    [sv(f"{side}_knee_servo", J("knee"), (0, 0, 0), "x"),
                     Geom(f"{side}_shin", (18, 18, SHIN), (0, 0, -SHIN / 2), 11, "shell")],
                    [Body(f"{side}_ankle_pitch", (0, 0, -SHIN),
                        Joint(f"{side}_ankle_pitch", (0, 1, 0), (-45, 45), J("ankle_pitch")),
                        [sv(f"{side}_ankle_pitch_servo", J("ankle_pitch"), (0, 0, 11), "x")],
                        [Body(f"{side}_ankle_roll", (0, 0, 0),
                            Joint(f"{side}_ankle_roll", (1, 0, 0), (-30, 30), J("ankle_roll")),
                            [sv(f"{side}_ankle_roll_servo", J("ankle_roll"), (-14, 0, -2), "y"),
                             Geom(f"{side}_foot", (72, 42, 8), (10, 0, -(ANKLE_Z - 4)), 12, "frame")])])])])])])


def arm(side):
    s = 1 if side == "l" else -1
    roll_range = (-10, 160) if s > 0 else (-160, 10)
    return Body(f"{side}_shoulder_pitch", (0, 40 * s, 97),
        Joint(f"{side}_shoulder_pitch", (0, 1, 0), (-180, 60), "MG90S"),
        [Geom(f"{side}_shoulder_cap", (18, 14, 28), (0, 5 * s, 0), 1, "shell")],
        [Body(f"{side}_shoulder_roll", (0, 5 * s, 0),
            Joint(f"{side}_shoulder_roll", (1, 0, 0), roll_range, "SG90"),
            [sv(f"{side}_shoulder_roll_servo", "SG90", (0, 0, -6), "z"),
             Geom(f"{side}_upper_arm", (16, 16, 36), (0, 0, -30), 3, "shell")],
            [Body(f"{side}_elbow", (0, 0, -50),
                Joint(f"{side}_elbow", (0, 1, 0), (-130, 0), "SG90"),
                [sv(f"{side}_elbow_servo", "SG90", (0, 0, 0), "x"),
                 Geom(f"{side}_forearm", (14, 14, 38), (0, 0, -19), 2, "shell"),
                 Geom(f"{side}_hand", (14, 14, 14), (0, 0, -44), 1.5, "shell")])])])


HEAD = dict(size=(62, 74, 64), pos=(2, 0, 24), mass=14)   # "A" shell: big rounded helmet, ~1/4 of height

# Base frame sits at the hip point H. Everything above is measured from there.
ROBOT = Body("base", (0, 0, HIP_Z), None, [
    Geom("pelvis", (40, 68, 36), (0, 0, 21), 10, "frame"),              # holds both hip-yaw servos
    Geom("torso_shell", (46, 74, 72), (0, 0, 74), 20, "shell"),
    Geom("battery_2s", (30, 56, 19), (0, 0, 49.5), 57, "battery"),     # flat, centred, just above the pelvis
    Geom("electronics", (25, 62, 25), (0, 0, 75), 49, "pcb"),
    Geom("ubec", (8, 26, 46), (-17, 0, 82), 15, "pcb"),
    sv("l_shoulder_pitch_servo", "MG90S", (0, 23.5, 96), "y"),
    sv("r_shoulder_pitch_servo", "MG90S", (0, -23.5, 96), "y"),
    sv("neck_yaw_servo", "SG90", (0, 0, 98), "z"),
    Geom("wiring", (30, 50, 40), (0, 0, 58), 30, "hidden"),
], [
    leg("l"), leg("r"), arm("l"), arm("r"),
    Body("neck_yaw", (0, 0, 110),
        Joint("neck_yaw", (0, 0, 1), (-90, 90), "SG90"),
        [Geom("neck", (16, 16, 10), (0, 0, 5), 4, "frame"),
         sv("head_pitch_servo", "SG90", (0, 0, 22), "y")],
        [Body("head_pitch", (0, 0, 22),
            Joint("head_pitch", (0, 1, 0), (-30, 45), "SG90"),
            [Geom("head_shell", HEAD["size"], HEAD["pos"], HEAD["mass"], "shell"),
             Geom("visor", (2, 58, 26), (34, 0, 24), 0, "lens"),
             Geom("camera", (6, 25, 24), (28, 0, 24), 6, "pcb")])]),
])


# ---------------------------------------------------------------- helpers
def m(v):
    return " ".join(f"{x / 1000:.4f}" for x in v)


def rgba(c):
    return " ".join(f"{x:g}" for x in C[c])


def walk(body):
    yield body
    for c in body.children:
        yield from walk(c)


# ---------------------------------------------------------------- MJCF
def mjcf():
    out = []

    def emit(body, indent):
        p = "  " * indent
        out.append(f'{p}<body name="{body.name}" pos="{m(body.pos)}">')
        if body.joint is None:
            out.append(f'{p}  <freejoint name="root"/>')
        else:
            j = body.joint
            out.append(f'{p}  <joint name="{j.name}" axis="{" ".join(map(str, j.axis))}" '
                       f'range="{j.range_deg[0]} {j.range_deg[1]}"/>')
        for g in body.geoms:
            half = " ".join(f"{s / 2000:.4f}" for s in g.size)
            cls = "hidden" if g.color == "hidden" else ("foot" if g.name.endswith("foot") else "part")
            out.append(f'{p}  <geom class="{cls}" name="{g.name}" type="box" size="{half}" '
                       f'pos="{m(g.pos)}" mass="{g.mass / 1000:.4f}" rgba="{rgba(g.color)}"/>')
        for c in body.children:
            emit(c, indent + 1)
        out.append(f"{p}</body>")

    emit(ROBOT, 2)
    joints = [b.joint for b in walk(ROBOT) if b.joint]
    acts = "\n".join(
        f'    <position name="{j.name}" joint="{j.name}" kp="{KP}" '
        f'forcerange="-{SERVOS[j.servo]["stall"]} {SERVOS[j.servo]["stall"]}" '
        f'ctrlrange="{j.range_deg[0] * pi / 180:.4f} {j.range_deg[1] * pi / 180:.4f}"/>'
        for j in joints)
    sensors = "\n".join([
        '    <framequat name="imu_quat" objtype="body" objname="base"/>',
        '    <gyro name="imu_gyro" site="imu"/>',
        '    <accelerometer name="imu_acc" site="imu"/>',
        '    <subtreecom name="com" body="base"/>',
    ])
    body_xml = "\n".join(out)
    # IMU site inside the torso, next to the electronics stack
    body_xml = body_xml.replace('<freejoint name="root"/>',
                                '<freejoint name="root"/>\n      <site name="imu" pos="0 0 0.050"/>', 1)
    return f"""<mujoco model="mini_humanoid">
  <!-- Generated by gen_robot.py — edit that file, not this one. -->
  <compiler angle="degree" autolimits="true"/>
  <option timestep="0.002" integrator="implicitfast"/>
  <visual><global offwidth="1280" offheight="720"/></visual>

  <default>
    <joint damping="{DAMPING}" armature="{ARMATURE}"/>
    <default class="part"><geom contype="2" conaffinity="1" friction="0.8"/></default>
    <default class="foot"><geom contype="2" conaffinity="1" friction="1.0"/></default>
    <default class="hidden"><geom contype="0" conaffinity="0" group="3"/></default>
  </default>

  <asset>
    <texture name="grid" type="2d" builtin="checker" width="512" height="512"
             rgb1="0.82 0.82 0.80" rgb2="0.74 0.74 0.72"/>
    <material name="grid" texture="grid" texrepeat="8 8" reflectance="0.05"/>
  </asset>

  <worldbody>
    <light pos="0.3 0.3 1.2" dir="-0.3 -0.3 -1" diffuse="0.9 0.9 0.9"/>
    <geom name="floor" type="plane" size="1 1 0.01" material="grid" contype="1" conaffinity="1"/>
{body_xml}
  </worldbody>

  <actuator>
{acts}
  </actuator>

  <sensor>
{sensors}
  </sensor>

  <keyframe>
    <key name="stand" qpos="0 0 {ROBOT.pos[2] / 1000:.4f} 1 0 0 0 {" ".join(["0"] * len(joints))}"
         ctrl="{" ".join(["0"] * len(joints))}"/>
  </keyframe>
</mujoco>
"""


# ---------------------------------------------------------------- URDF
def box_inertia(mass, size):
    x, y, z = size
    return (mass / 12 * (y * y + z * z), mass / 12 * (x * x + z * z), mass / 12 * (x * x + y * y))


def link_inertial(geoms):
    """Combine box geoms into one inertial (SI units), parallel-axis about the link CoM."""
    gs = [(g.mass / 1000, [s / 1000 for s in g.size], [p / 1000 for p in g.pos]) for g in geoms if g.mass > 0]
    M = sum(g[0] for g in gs) or 1e-4
    com = [sum(g[0] * g[2][i] for g in gs) / M for i in range(3)] if gs else [0, 0, 0]
    ixx = iyy = izz = 0.0
    for mass, size, pos in gs:
        a, b, c = box_inertia(mass, size)
        dx, dy, dz = (pos[i] - com[i] for i in range(3))
        ixx += a + mass * (dy * dy + dz * dz)
        iyy += b + mass * (dx * dx + dz * dz)
        izz += c + mass * (dx * dx + dy * dy)
    return M, com, (max(ixx, 1e-8), max(iyy, 1e-8), max(izz, 1e-8))


def urdf():
    out = ['<?xml version="1.0"?>',
           '<!-- Generated by gen_robot.py — edit that file, not this one. -->',
           '<robot name="mini_humanoid">']
    mats = set()
    for b in walk(ROBOT):
        for g in b.geoms:
            mats.add(g.color)
    for c in sorted(mats):
        out.append(f'  <material name="{c}"><color rgba="{rgba(c)}"/></material>')

    for b in walk(ROBOT):
        link = "base_link" if b.joint is None else b.name + "_link"
        M, com, inertia = link_inertial(b.geoms)
        out.append(f'  <link name="{link}">')
        out.append(f'    <inertial><origin xyz="{" ".join(f"{v:.4f}" for v in com)}"/>'
                   f'<mass value="{M:.4f}"/>'
                   f'<inertia ixx="{inertia[0]:.3e}" iyy="{inertia[1]:.3e}" izz="{inertia[2]:.3e}" '
                   f'ixy="0" ixz="0" iyz="0"/></inertial>')
        for g in b.geoms:
            if g.color == "hidden":
                continue
            out.append(f'    <visual name="{g.name}"><origin xyz="{m(g.pos)}"/>'
                       f'<geometry><box size="{m(g.size)}"/></geometry><material name="{g.color}"/></visual>')
            out.append(f'    <collision><origin xyz="{m(g.pos)}"/>'
                       f'<geometry><box size="{m(g.size)}"/></geometry></collision>')
        out.append("  </link>")
    for b in walk(ROBOT):
        parent = "base_link" if b.joint is None else b.name + "_link"
        for c in b.children:
            j = c.joint
            lo, hi = (v * pi / 180 for v in j.range_deg)
            sv = SERVOS[j.servo]
            vel = (pi / 3) / sv["speed"]
            out.append(f'  <joint name="{j.name}" type="revolute">'
                       f'<parent link="{parent}"/><child link="{c.name}_link"/>'
                       f'<origin xyz="{m(c.pos)}"/><axis xyz="{" ".join(map(str, j.axis))}"/>'
                       f'<limit lower="{lo:.4f}" upper="{hi:.4f}" effort="{sv["stall"]}" '
                       f'velocity="{vel:.2f}"/></joint>')
    out.append("</robot>")
    return "\n".join(out) + "\n"


if __name__ == "__main__":
    here = Path(__file__).parent
    (here / "humanoid.xml").write_text(mjcf(), encoding="utf-8")
    (here / "humanoid.urdf").write_text(urdf(), encoding="utf-8")
    joints = [b.joint for b in walk(ROBOT) if b.joint]
    mass = sum(g.mass for b in walk(ROBOT) for g in b.geoms)
    print(f"wrote humanoid.xml + humanoid.urdf: {len(joints)} joints, {mass:.0f} g")
