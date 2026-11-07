"""Generate MJCF (MuJoCo) and URDF (Foxglove / ROS 2) for the 20-DoF mini humanoid (v0.5).

Single source of truth: every size, mass and joint below comes from the design doc
"Mini Humanoid - Thiết kế phần cứng". Edit here, re-run, both files update together.

    python gen_robot.py            -> humanoid.xml, humanoid.urdf

Units in this file: millimetres and grams. Output files use metres and kilograms.
Frame: x forward, y left, z up. Feet soles touch z = 0 in the default pose.
"""
import os
from dataclasses import dataclass, field
from math import cos, pi, sin
from pathlib import Path

# ---------------------------------------------------------------- servos
# Stall torque (N.m) is the hard limit given to the simulator.
# Firmware should keep continuous load below ~1/3 of stall.
SERVOS = {
    # heavy joints: hip roll/pitch, knee, ankle pitch/roll (sold in HCMC, Hshop / icdayroi)
    "MG92B": dict(stall=0.343, speed=0.08, mass=14.0, size=(22.8, 12, 31)),   # TowerPro, 3.5 kg.cm @6V, metal gear
    # medium: hip yaw, shoulder pitch (metal gears survive falls)
    "MG90S": dict(stall=0.216, speed=0.08, mass=13.4, size=(22.8, 12.2, 28.5)),  # 2.2 kg.cm
    # light: shoulder roll, elbow, head
    "SG90": dict(stall=0.177, speed=0.10, mass=9.0, size=(23, 12.2, 29)),      # 1.8 kg.cm @4.8V, plastic gear
    # stronger import-only option, kept for comparison runs (LEG_HEAVY=DS843MG)
    "DS843MG": dict(stall=0.471, speed=0.10, mass=8.5, size=(23, 9, 23)),     # Corona, 4.8 kg.cm @6V
}
LEG_HEAVY = os.environ.get("LEG_HEAVY", "MG92B")
LEG_SERVO = {}
# Feet sit 6 mm outboard of the ankle axis: with them centred the two soles (42 wide, 44 apart)
# rubbed together while walking and turning. 6 mm leaves >= 3.8 mm clearance in demo.py.
FOOT_OUT = 6.0
# Foot FSRs (FSR402, heel + toe). Box-plane contacts sit at the sole corners, so each touch
# site covers half the sole and sums the force on that half: front half = toe, rear half = heel.
FSR_TOE_X = 28.0    # site centre, mm forward of the ankle axis
FSR_HEEL_X = -8.0   # site centre, mm from the ankle axis (negative = behind)
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
                             Geom(f"{side}_foot", (72, 42, 8), (10, FOOT_OUT * s, -(ANKLE_Z - 4)), 12,
                                  "frame")])])])])])])


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


# ---------------------------------------------------------------- sensor and tool frames
# (name, parent body, xyz mm in the parent body frame, rpy rad). Every body frame is parallel to base_link
# at the zero pose (REP-103: x forward, y left, z up), so these are also the mounting directions.
CAM_TILT = 15 * pi / 180   # camera pitched down so the floor is visible from ~0.35 m
TOF_TILT = 20 * pi / 180   # ToF pitched down so its beam meets the floor at 0.46-2.2 m
FRAMES = [
    ("imu_link", "base", (0, 0, 30), (0, 0, 0)),             # MPU6050 on the pelvis top plate, under the battery
    ("camera_link", "head_pitch", (31, 0, 42), (0, CAM_TILT, 0)),
    ("tof_link", "head_pitch", (31, 8, 45), (0, TOF_TILT, 0)),
    ("l_mic_link", "head_pitch", (2, 38.5, 24), (0, 0, pi / 2)),   # mic faces +y (out of the left ear)
    ("r_mic_link", "head_pitch", (2, -38.5, 24), (0, 0, -pi / 2)),
    ("l_sole", "l_ankle_roll", (0, 0, -ANKLE_Z), (0, 0, 0)),       # REP-120: on the sole, under the ankle axis
    ("r_sole", "r_ankle_roll", (0, 0, -ANKLE_Z), (0, 0, 0)),
    ("l_gripper", "l_elbow", (0, 0, -44), (0, 0, 0)),
    ("r_gripper", "r_elbow", (0, 0, -44), (0, 0, 0)),
]
# camera_optical_frame (z forward, x right, y down) hangs off camera_link in the URDF only.
OPTICAL_RPY = (-pi / 2, 0, -pi / 2)


# ---------------------------------------------------------------- helpers
def cos_sin(a):
    return cos(a), sin(a)


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
            if cls == "foot":
                # FSR402 heel / toe, read as MuJoCo touch sensors (see FSR_TOE_X)
                side = g.name.split("_")[0]
                sole_z = (g.pos[2] - g.size[2] / 2) / 1000
                for pad, x in (("toe", FSR_TOE_X), ("heel", FSR_HEEL_X)):
                    out.append(f'{p}  <site name="{side}_fsr_{pad}" type="box" size="0.0181 0.0215 0.003" '
                               f'pos="{x / 1000:.4f} {g.pos[1] / 1000:.4f} {sole_z:.4f}"/>')
        for name, parent, xyz, rpy in FRAMES:
            if parent == body.name:
                eul = " ".join(f"{a * 180 / pi:.3f}" for a in rpy)
                out.append(f'{p}  <site name="{name}" pos="{m(xyz)}" euler="{eul}"/>')
        if body.name == "head_pitch":
            # MuJoCo rangefinders measure along the site z axis, so the ToF beam gets its own site
            out.append(f'{p}  <site name="tof_beam" pos="{m(FRAMES[2][2])}" '
                       f'euler="0 {90 + TOF_TILT * 180 / pi:.3f} 0"/>')
            c, sn = cos_sin(CAM_TILT)
            out.append(f'{p}  <camera name="head_cam" pos="{m(FRAMES[1][2])}" fovy="48.8" '
                       f'xyaxes="0 -1 0 {sn:.4f} 0 {c:.4f}"/>')
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
        '    <gyro name="imu_gyro" site="imu_link"/>',
        '    <accelerometer name="imu_acc" site="imu_link"/>',
        '    <rangefinder name="tof" site="tof_beam" cutoff="2.0"/>',
        '    <subtreecom name="com" body="base"/>',
    ] + [f'    <touch name="{side}_fsr_{pad}" site="{side}_fsr_{pad}"/>'
         for side in ("l", "r") for pad in ("toe", "heel")])
    body_xml = "\n".join(out)
    return f"""<mujoco model="mini_humanoid">
  <!-- Generated by gen_robot.py — edit that file, not this one. -->
  <compiler angle="degree" autolimits="true"/>
  <option timestep="0.002" integrator="implicitfast"/>
  <visual><global offwidth="1280" offheight="720"/></visual>

  <default>
    <joint damping="{DAMPING}" armature="{ARMATURE}"/>
    <default class="part"><geom contype="2" conaffinity="1" friction="0.8"/></default>
    <default class="foot"><geom contype="2" conaffinity="3" friction="1.0"/></default>
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
           '<robot name="mini_humanoid">',
           '  <!-- Frames follow REP-103 (x forward, y left, z up) and REP-120 names (imu_link, l_sole, ...).',
           '       base_link sits at the hip point H, 115 mm above the floor in the standing pose. -->',
           '  <!-- MuJoCo reads this tag when the URDF is loaded directly; other tools ignore it. -->',
           '  <mujoco><compiler discardvisual="false" fusestatic="false" balanceinertia="true"/></mujoco>']
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
            out.append(f'    <collision name="{g.name}_collision"><origin xyz="{m(g.pos)}"/>'
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
                       f'velocity="{vel:.2f}"/><dynamics damping="{DAMPING}" friction="0"/></joint>')
    # sensor and tool frames: massless links on fixed joints
    for name, parent, xyz, rpy in FRAMES:
        plink = "base_link" if parent == "base" else parent + "_link"
        out.append(f'  <link name="{name}"/>')
        out.append(f'  <joint name="{name}_joint" type="fixed"><parent link="{plink}"/><child link="{name}"/>'
                   f'<origin xyz="{m(xyz)}" rpy="{" ".join(f"{a:.6f}" for a in rpy)}"/></joint>')
    out.append('  <link name="camera_optical_frame"/>')
    out.append('  <joint name="camera_optical_joint" type="fixed"><parent link="camera_link"/>'
               '<child link="camera_optical_frame"/>'
               f'<origin xyz="0 0 0" rpy="{" ".join(f"{a:.6f}" for a in OPTICAL_RPY)}"/></joint>')
    out.append("</robot>")
    return "\n".join(out) + "\n"


# ---------------------------------------------------------------- Gazebo (gz-sim Harmonic) xacro
IMU_NOISE = dict(gyro=8.7e-4, acc=0.039)   # MPU6050 datasheet: 0.005 deg/s/rtHz and 400 ug/rtHz, ~100 Hz bandwidth


def _noise(sd):
    return f'<noise type="gaussian"><mean>0</mean><stddev>{sd}</stddev></noise>'


def gz_xacro():
    body = [ln for ln in urdf().splitlines()[3:-1] if "<mujoco>" not in ln and "MuJoCo reads" not in ln]
    joints = [b.joint for b in walk(ROBOT) if b.joint]
    ctrl = "\n".join(
        f'    <joint name="{j.name}"><command_interface name="position"/>'
        f'<state_interface name="position"><param name="initial_value">0.0</param></state_interface>'
        f'<state_interface name="velocity"/><state_interface name="effort"/></joint>' for j in joints)
    axes = "".join(f"<{a}>{_noise(IMU_NOISE[k])}</{a}>" for a in ("x", "y", "z") for k in ("gyro",))
    lin = "".join(f"<{a}>{_noise(IMU_NOISE['acc'])}</{a}>" for a in ("x", "y", "z"))
    feet = "\n".join(f'  <gazebo reference="{s}_ankle_roll_link"><mu1>1.0</mu1><mu2>1.0</mu2></gazebo>' for s in "lr")
    return f"""<?xml version="1.0"?>
<!-- Generated by sim/mini_humanoid/gen_robot.py — edit that file, not this one.
     Mico for Gazebo Harmonic (gz-sim) + ros2_control. Same links, joints and frames as humanoid.urdf. -->
<robot name="mico" xmlns:xacro="http://www.ros.org/wiki/xacro">
  <xacro:arg name="controllers" default="$(find humanoid_description)/config/mico_controllers.yaml"/>
{chr(10).join(body)}

  <ros2_control name="mico" type="system">
    <hardware><plugin>gz_ros2_control/GazeboSimSystem</plugin></hardware>
{ctrl}
  </ros2_control>

  <gazebo>
    <plugin filename="gz_ros2_control-system" name="gz_ros2_control::GazeboSimROS2ControlPlugin">
      <parameters>$(arg controllers)</parameters>
    </plugin>
    <plugin filename="gz-sim-imu-system" name="gz::sim::systems::Imu"/>
    <plugin filename="gz-sim-sensors-system" name="gz::sim::systems::Sensors">
      <render_engine>ogre2</render_engine>
    </plugin>
  </gazebo>

  <gazebo reference="imu_link">
    <sensor name="imu" type="imu">
      <always_on>1</always_on><update_rate>200</update_rate><topic>imu</topic><gz_frame_id>imu_link</gz_frame_id>
      <imu><angular_velocity>{axes}</angular_velocity><linear_acceleration>{lin}</linear_acceleration></imu>
    </sensor>
  </gazebo>

  <gazebo reference="camera_link">
    <sensor name="head_camera" type="camera">
      <always_on>1</always_on><update_rate>30</update_rate><topic>camera/image_raw</topic>
      <gz_frame_id>camera_optical_frame</gz_frame_id>
      <camera><horizontal_fov>1.0856</horizontal_fov>
        <image><width>640</width><height>480</height><format>R8G8B8</format></image>
        <clip><near>0.02</near><far>10</far></clip><optical_frame_id>camera_optical_frame</optical_frame_id></camera>
    </sensor>
  </gazebo>

  <gazebo reference="tof_link">
    <sensor name="tof" type="gpu_lidar">
      <always_on>1</always_on><update_rate>30</update_rate><topic>tof</topic><gz_frame_id>tof_link</gz_frame_id>
      <lidar><scan><horizontal><samples>1</samples><resolution>1</resolution>
        <min_angle>0</min_angle><max_angle>0</max_angle></horizontal></scan><range><min>0.03</min><max>2.0</max><resolution>0.001</resolution></range></lidar>
    </sensor>
  </gazebo>

{feet}
</robot>
"""


if __name__ == "__main__":
    here = Path(__file__).parent
    (here / "humanoid.xml").write_text(mjcf(), encoding="utf-8")
    (here / "humanoid.urdf").write_text(urdf(), encoding="utf-8")
    desc = here.parents[1] / "src" / "humanoid_description" / "urdf"
    if desc.is_dir():
        (desc / "mico.urdf.xacro").write_text(gz_xacro(), encoding="utf-8")
    joints = [b.joint for b in walk(ROBOT) if b.joint]
    mass = sum(g.mass for b in walk(ROBOT) for g in b.geoms)
    print(f"wrote humanoid.xml + humanoid.urdf: {len(joints)} joints, {mass:.0f} g")
