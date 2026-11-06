"""Quasi-static walk, turn in place and arm wave for the 20-DoF mini humanoid (MuJoCo).

    mjpython demo.py            # macOS: interactive viewer (needs mjpython)
    python demo.py              # Linux/Windows: interactive viewer
    python demo.py --check      # headless: run the whole sequence, print stability + servo load report

Every step is the same 4-phase cycle: shift the CoM over the stance foot (hip roll +
ankle roll), lift and move the swing foot, put it down, shift back. Turning reuses the
cycle with the hip-yaw joints. All targets go to position actuators that are
torque-limited at the servo stall torque.
"""
import argparse
import math
import time

import mujoco
import mujoco.viewer
import numpy as np

# ---------------------------------------------------------------- gait parameters
L1 = L2 = 0.045      # thigh, shin (m): hip point H -> knee -> ankle point A
H0 = 0.087           # nominal H-to-A height (slight crouch)
STEP = 0.024         # step length (m)
LIFT = 0.010         # foot clearance (m)
SWAY = 0.20          # hip/ankle roll during single support (rad), shifts the CoM over the stance foot
LEAN = -0.10         # extra torso lean toward the stance leg (rad, negative = toward it); unloads the stance hip roll
T_STEP = 1.6         # seconds per step
WALK_STEPS = 8
TURN = math.radians(20)   # body rotation per pair of turning steps
TURN_PAIRS = 3            # 3 x 20 deg = 60 deg to the left


def leg_ik(x, z):
    """Sagittal IK. Ankle point A at x forward, z below hip point H (leg frame, after hip yaw).
    Returns hip_pitch, knee, ankle_pitch that keep the foot flat."""
    d = min(math.hypot(x, z), L1 + L2 - 1e-6)
    alpha = math.atan2(x, z)
    beta = math.acos(d / (2 * L1))
    return -(alpha + beta), 2 * beta, alpha - beta


def smooth(u):
    u = min(max(u, 0.0), 1.0)
    return u * u * (3 - 2 * u)


def step_pose(u, swing, x0, x1, yaw_sw, yaw_st):
    """One quasi-static step at phase u in [0, 1].
    swing: 'l' or 'r'. x0 -> x1: swing-foot x relative to its hip (stance mirrors it).
    yaw_sw, yaw_st: (start, end) hip-yaw angles of swing and stance leg."""
    stance = "l" if swing == "r" else "r"
    q = {}
    s = smooth(u / 0.25) if u < 0.25 else (1.0 if u < 0.75 else 1 - smooth((u - 0.75) / 0.25))
    sign = 1 if stance == "l" else -1
    r = SWAY * s * sign
    lean = LEAN * s * sign
    v = smooth((u - 0.25) / 0.5)              # swing happens only in single support
    xs = x0 + (x1 - x0) * v
    zs = H0 - LIFT * math.sin(math.pi * v)
    xt = -x0 + (x0 - x1) * v
    for side, (x, z), (y0, y1) in ((swing, (xs, zs), yaw_sw), (stance, (xt, H0), yaw_st)):
        q[f"{side}_hip_pitch"], q[f"{side}_knee"], q[f"{side}_ankle_pitch"] = leg_ik(x, z)
        q[f"{side}_ankle_roll"], q[f"{side}_hip_roll"] = r, -r - lean
        q[f"{side}_hip_yaw"] = y0 + (y1 - y0) * v
    return q


def walk(t):
    k = min(int(t // T_STEP), WALK_STEPS - 1)
    u = 1.0 if t >= WALK_STEPS * T_STEP else (t % T_STEP) / T_STEP
    swing = "r" if k % 2 == 0 else "l"
    x0 = 0.0 if k == 0 else -STEP / 2
    x1 = 0.0 if k == WALK_STEPS - 1 else STEP / 2
    q = step_pose(u, swing, x0, x1, (0, 0), (0, 0))
    q["l_shoulder_pitch"] = 0.25 * math.sin(math.pi * (k + u))
    q["r_shoulder_pitch"] = -q["l_shoulder_pitch"]
    q["l_elbow"] = q["r_elbow"] = -0.4
    return q


def turn(t):
    """Turn left in place. Pair = (1) lift left foot, open its hip yaw to +TURN;
    (2) lift right foot while the left hip yaw returns to 0: the planted left foot
    stays put, so the whole body rotates +TURN."""
    n = 2 * TURN_PAIRS
    k = min(int(t // T_STEP), n - 1)
    u = 1.0 if t >= n * T_STEP else (t % T_STEP) / T_STEP
    if k % 2 == 0:
        q = step_pose(u, "l", 0, 0, (0, TURN), (0, 0))
    else:
        q = step_pose(u, "r", 0, 0, (0, 0), (TURN, 0))
    q["l_elbow"] = q["r_elbow"] = -0.4
    return q


def wave(t):
    q = {f"{s}_{j}": a for s in "lr" for j, a in zip(("hip_pitch", "knee", "ankle_pitch"), leg_ik(0, H0))}
    q["r_shoulder_pitch"] = -2.6
    q["r_shoulder_roll"] = -0.3
    q["r_elbow"] = -0.5 + 0.3 * math.sin(2 * math.pi * 1.0 * t)
    q["l_elbow"] = -0.4
    q["neck_yaw"] = 0.4 * math.sin(2 * math.pi * 0.25 * t)
    return q


def crouch(t):
    w = smooth(t / 1.0)
    hp, kn, ap = leg_ik(0, 0.090 + (H0 - 0.090) * w)
    q = {f"{s}_{j}": a for s in "lr" for j, a in zip(("hip_pitch", "knee", "ankle_pitch"), (hp, kn, ap))}
    q["l_elbow"] = q["r_elbow"] = -0.4 * w
    return q


def blend(a, b, w):
    keys = set(a) | set(b)
    return {k: (1 - w) * a.get(k, 0.0) + w * b.get(k, 0.0) for k in keys}


# sequence: crouch -> walk -> pause -> turn -> pause -> wave
T_WALK0 = 1.0
T_TURN0 = T_WALK0 + WALK_STEPS * T_STEP + 0.5
T_WAVE0 = T_TURN0 + 2 * TURN_PAIRS * T_STEP + 0.5
T_END = T_WAVE0 + 5.0


def targets(t):
    if t < T_WALK0:
        return crouch(t)
    if t < T_TURN0:
        return walk(t - T_WALK0)
    if t < T_WAVE0:
        return turn(t - T_TURN0)
    return blend(turn(1e9), wave(t - T_WAVE0), smooth((t - T_WAVE0) / 1.5))


def controller(m, d, ids):
    for name, val in targets(d.time).items():
        d.ctrl[ids[name]] = val


def tilt_deg(d):
    """Angle between the torso's up axis and world up."""
    w, x, y, z = d.qpos[3:7]
    up_z = 1 - 2 * (x * x + y * y)
    return math.degrees(math.acos(max(-1.0, min(1.0, up_z))))


def yaw_deg(d):
    w, x, y, z = d.qpos[3:7]
    return math.degrees(math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z)))


def check(m, d, ids):
    load = np.zeros(m.nu)
    hot = np.zeros(m.nu)               # time above 1/3 stall (continuous rating)
    max_tilt = 0.0
    marks = {}
    while d.time < T_END:
        controller(m, d, ids)
        mujoco.mj_step(m, d)
        r = np.abs(d.actuator_force) / m.actuator_forcerange[:, 1]
        load = np.maximum(load, r)
        hot += (r > 1 / 3) * m.opt.timestep
        max_tilt = max(max_tilt, tilt_deg(d))
        for name, tm in (("walk_end", T_TURN0), ("turn_end", T_WAVE0)):
            if name not in marks and d.time >= tm:
                marks[name] = (d.qpos[0] * 1000, d.qpos[1] * 1000, yaw_deg(d))
    fell = d.qpos[2] < 0.09 or max_tilt > 30
    wx, wy, wyaw = marks["walk_end"]
    tx, ty, tyaw = marks["turn_end"]
    print(f"result      : {'FELL' if fell else 'OK, still standing'}")
    print(f"walk        : {wx:.0f} mm forward, {wy:+.0f} mm sideways, heading {wyaw:+.1f} deg")
    print(f"turn        : heading {wyaw:+.1f} -> {tyaw:+.1f} deg (target +{math.degrees(TURN) * TURN_PAIRS:.0f}),"
          f" drift {math.hypot(tx - wx, ty - wy):.0f} mm")
    print(f"max tilt    : {max_tilt:.1f} deg")
    print("servo peak load (fraction of stall) / seconds above 1/3 stall:")
    for i in np.argsort(-load)[:8]:
        print(f"  {m.actuator(i).name:18s} {load[i]:.2f}  {hot[i]:.2f}s")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--check", action="store_true", help="headless run with a report")
    args = ap.parse_args()
    m = mujoco.MjModel.from_xml_path("humanoid.xml")
    d = mujoco.MjData(m)
    mujoco.mj_resetDataKeyframe(m, d, 0)
    ids = {m.actuator(i).name: i for i in range(m.nu)}
    if args.check:
        check(m, d, ids)
        return
    with mujoco.viewer.launch_passive(m, d) as v:
        while v.is_running():
            t0 = time.time()
            controller(m, d, ids)
            mujoco.mj_step(m, d)
            v.sync()
            dt = m.opt.timestep - (time.time() - t0)
            if dt > 0:
                time.sleep(dt)


if __name__ == "__main__":
    main()
