"""Kinematics + dynamics tables for the 20-DoF mini humanoid (v0.5).

    python kin_dyn.py          # verify DH vs MuJoCo, run the gait, write tables/*.csv, print markdown

1. DH tables (standard / distal convention):  T_i = Rz(theta_i) . Tz(d_i) . Tx(a_i) . Rx(alpha_i)
   theta_i = q_i + offset_i.  Lengths in mm, angles in degrees.
   Each chain starts from a fixed base transform measured from the base frame, which sits
   at the hip point H (between both hips, 115 mm above the floor). Frame: x fwd, y left, z up.
2. Forward kinematics from the DH tables is checked against MuJoCo at random joint angles.
3. Link dynamics (mass, CoM, inertia) are read from the compiled MuJoCo model.
4. Joint loads come from the same walk + turn + wave run as demo.py, plus static gravity cases.
"""
import csv
import math
from pathlib import Path

import mujoco
import numpy as np

import demo
from gen_robot import SERVOS

OUT = Path(__file__).parent / "tables"

# ---------------------------------------------------------------- DH tables
# (joint, theta offset deg, d mm, a mm, alpha deg)


def leg_dh(s):
    return [
        (f"{s}_hip_yaw",     90, 0,  0,  90),
        (f"{s}_hip_roll",   -90, 0,  0, -90),
        (f"{s}_hip_pitch",    0, 0, 45,   0),
        (f"{s}_knee",         0, 0, 45,   0),
        (f"{s}_ankle_pitch",  0, 0,  0,  90),
        (f"{s}_ankle_roll",   0, 0, 25,   0),
    ]


def arm_dh(s):
    k = 1 if s == "l" else -1
    return [
        (f"{s}_shoulder_pitch", 0, 5 * k,  0,  90),
        (f"{s}_shoulder_roll",  0, 0,     50, -90),
        (f"{s}_elbow",          0, 0,     44,   0),
    ]


HEAD_DH = [
    ("neck_yaw",   0, 22, 0, -90),
    ("head_pitch", 0,  0, 0,   0),
]

# fixed base -> frame 0 of each chain (rotation columns = x0, y0, z0 in the base frame)
R_ARM0 = np.array([[0, -1, 0], [0, 0, 1], [-1, 0, 0]], float)   # x0 = -z, y0 = -x, z0 = +y
CHAINS = {
    "l_leg": dict(dh=leg_dh("l"), p0=(0, 22, 0), R0=np.eye(3), tool=None, tip="sole (dưới A)"),
    "r_leg": dict(dh=leg_dh("r"), p0=(0, -22, 0), R0=np.eye(3), tool=None, tip="sole (dưới A)"),
    "l_arm": dict(dh=arm_dh("l"), p0=(0, 40, 97), R0=R_ARM0, tool=None, tip="tâm bàn tay"),
    "r_arm": dict(dh=arm_dh("r"), p0=(0, -40, 97), R0=R_ARM0, tool=None, tip="tâm bàn tay"),
    "head":  dict(dh=HEAD_DH, p0=(0, 0, 110), R0=np.eye(3), tool=(28, -24, 0), tip="camera"),
}


def dh_T(theta, d, a, alpha):
    ct, st, ca, sa = math.cos(theta), math.sin(theta), math.cos(alpha), math.sin(alpha)
    return np.array([[ct, -st * ca, st * sa, a * ct],
                     [st, ct * ca, -ct * sa, a * st],
                     [0, sa, ca, d],
                     [0, 0, 0, 1]])


def fk(chain, q):
    """Return list of frame origins (mm, base frame) for frames 0..n (+ tool)."""
    c = CHAINS[chain]
    T = np.eye(4)
    T[:3, :3] = c["R0"]
    T[:3, 3] = c["p0"]
    pts = [T[:3, 3].copy()]
    for (name, off, d, a, al) in c["dh"]:
        T = T @ dh_T(q[name] + math.radians(off), d, a, math.radians(al))
        pts.append(T[:3, 3].copy())
    if c["tool"] is not None:
        pts.append((T @ np.array([*c["tool"], 1.0]))[:3])
    return pts, T


# MuJoCo reference points for the same frames: (body, local offset in mm)
MJ_POINTS = {
    "l_leg": [("l_knee", (0, 0, 0)), ("l_ankle_roll", (0, 0, 0)), ("l_ankle_roll", (0, 0, -25))],
    "r_leg": [("r_knee", (0, 0, 0)), ("r_ankle_roll", (0, 0, 0)), ("r_ankle_roll", (0, 0, -25))],
    "l_arm": [("l_elbow", (0, 0, 0)), ("l_elbow", (0, 0, -44))],
    "r_arm": [("r_elbow", (0, 0, 0)), ("r_elbow", (0, 0, -44))],
    "head":  [("head_pitch", (0, 0, 0)), ("head_pitch", (28, 0, 24))],
}
# which DH frame index each MuJoCo point corresponds to (frame list from fk: 0 = base->frame0)
DH_INDEX = {"l_leg": [3, 4, 6], "r_leg": [3, 4, 6], "l_arm": [2, 3], "r_arm": [2, 3], "head": [2, 3]}


def verify_dh(m, n=300, seed=1):
    rng = np.random.default_rng(seed)
    d = mujoco.MjData(m)
    worst = {}
    for _ in range(n):
        mujoco.mj_resetData(m, d)
        d.qpos[:3] = (0, 0, 0.115)          # base frame at the hip point, upright
        d.qpos[3:7] = (1, 0, 0, 0)
        q = {}
        for j in range(1, m.njnt):
            lo, hi = m.jnt_range[j]
            v = rng.uniform(lo, hi)
            d.qpos[m.jnt_qposadr[j]] = v
            q[m.joint(j).name] = v
        mujoco.mj_kinematics(m, d)
        for ch, refs in MJ_POINTS.items():
            pts, _ = fk(ch, q)
            for (body, off), idx in zip(refs, DH_INDEX[ch]):
                b = m.body(body).id
                p_mj = (d.xpos[b] + d.xmat[b].reshape(3, 3) @ (np.array(off) / 1000) - [0, 0, 0.115]) * 1000
                err = np.linalg.norm(p_mj - pts[idx])
                worst[ch] = max(worst.get(ch, 0), err)
    return worst


# ---------------------------------------------------------------- dynamics
def link_table(m):
    rows = []
    for b in range(1, m.nbody):
        rows.append(dict(
            link=m.body(b).name,
            mass_g=m.body_mass[b] * 1000,
            com_mm=" / ".join(f"{v:.1f}" for v in m.body_ipos[b] * 1000),
            inertia_gmm2=" / ".join(f"{v:.0f}" for v in m.body_inertia[b] * 1e9),   # principal, kg.m2 -> g.mm2
        ))
    return rows


def static_gravity(m):
    """Gravity torque per joint (N.m) for worst static poses, base held upright.
    qfrc_bias with zero velocity = gravity load carried by each joint."""
    d = mujoco.MjData(m)
    out = {}
    poses = {
        "tay duỗi ngang trước": dict(l_shoulder_pitch=-math.pi / 2, r_shoulder_pitch=-math.pi / 2),
        "tay dang ngang": dict(l_shoulder_roll=math.pi / 2, r_shoulder_roll=-math.pi / 2),
        "cúi đầu 45°": dict(head_pitch=math.radians(45)),
    }
    for label, pose in poses.items():
        mujoco.mj_resetData(m, d)
        d.qpos[:3] = (0, 0, 0.115)
        d.qpos[3:7] = (1, 0, 0, 0)
        for j, v in pose.items():
            d.qpos[m.jnt_qposadr[m.joint(j).id]] = v
        mujoco.mj_forward(m, d)
        for j in range(1, m.njnt):
            name = m.joint(j).name
            tau = abs(d.qfrc_bias[m.jnt_dofadr[j]])
            if tau > out.get(name, (0, ""))[0]:
                out[name] = (tau, label)
    return out


def static_hold(m, target, settle=2.0):
    """Ease into a pose standing on the floor (crouch 1 s, blend 1.5 s), hold it, return the
    actuator torque at the end (N.m). Velocities are ~0, so this is the static load."""
    d = mujoco.MjData(m)
    mujoco.mj_resetDataKeyframe(m, d, 0)
    ids = {m.actuator(i).name: i for i in range(m.nu)}
    t_end = 1.0 + 1.5 + settle
    while d.time < t_end:
        t = d.time
        q = demo.crouch(t) if t < 1.0 else demo.blend(demo.crouch(1.0), target, demo.smooth((t - 1.0) / 1.5))
        for k, v in q.items():
            d.ctrl[ids[k]] = v
        mujoco.mj_step(m, d)
    upright = d.qpos[2] > 0.09
    return {m.actuator(i).name: abs(d.actuator_force[i]) for i in range(m.nu)}, upright


def static_floor_cases(m):
    """Leg loads that only exist with the feet on the floor."""
    single = demo.step_pose(0.5, "r", 0, 0, (0, 0), (0, 0))        # on the left foot, right foot lifted
    single.update(l_elbow=-0.4, r_elbow=-0.4)
    hp, kn, ap = demo.leg_ik(0, 0.070)                              # squat: hip-to-ankle 70 mm
    squat = {f"{s}_{j}": a for s in "lr" for j, a in zip(("hip_pitch", "knee", "ankle_pitch"), (hp, kn, ap))}
    out = {}
    for label, pose in (("đứng một chân (trái)", single), (f"ngồi xổm, gối {math.degrees(kn):.0f}°", squat)):
        f, ok = static_hold(m, pose)
        assert ok, f"robot fell in static case {label}"
        for k, v in f.items():
            if v > out.get(k, (0, ""))[0]:
                out[k] = (v, label)
    return out


def gait_stats(m):
    d = mujoco.MjData(m)
    mujoco.mj_resetDataKeyframe(m, d, 0)
    ids = {m.actuator(i).name: i for i in range(m.nu)}
    jd = {m.actuator(i).name: m.jnt_dofadr[m.actuator_trnid[i, 0]] for i in range(m.nu)}
    peak_t = np.zeros(m.nu)
    sq = np.zeros(m.nu)
    peak_v = np.zeros(m.nu)
    peak_a = np.zeros(m.nu)
    n = 0
    prev_v = None
    while d.time < demo.T_END:
        demo.controller(m, d, ids)
        mujoco.mj_step(m, d)
        f = np.abs(d.actuator_force)
        v = np.array([d.qvel[jd[m.actuator(i).name]] for i in range(m.nu)])
        if prev_v is not None:
            peak_a = np.maximum(peak_a, np.abs(v - prev_v) / m.opt.timestep)
        prev_v = v
        peak_t = np.maximum(peak_t, f)
        sq += f * f
        peak_v = np.maximum(peak_v, np.abs(v))
        n += 1
    return {m.actuator(i).name: dict(peak=peak_t[i], rms=math.sqrt(sq[i] / n), vel=peak_v[i], acc=peak_a[i])
            for i in range(m.nu)}


# ---------------------------------------------------------------- main
def main():
    m = mujoco.MjModel.from_xml_path("humanoid.xml")
    OUT.mkdir(exist_ok=True)
    servo_of = {}
    import gen_robot
    for b in gen_robot.walk(gen_robot.ROBOT):
        if b.joint:
            servo_of[b.joint.name] = b.joint.servo

    # DH
    worst = verify_dh(m)
    with open(OUT / "dh_table.csv", "w", newline="", encoding="utf-8") as f:
        w = csv.writer(f)
        w.writerow(["chain", "i", "joint", "theta_i (deg)", "d_i (mm)", "a_i (mm)", "alpha_i (deg)", "servo"])
        for ch, c in CHAINS.items():
            for i, (name, off, dd, a, al) in enumerate(c["dh"], 1):
                th = "q" if off == 0 else f"q {'+' if off > 0 else '-'} {abs(off)}"
                w.writerow([ch, i, name, th, dd, a, al, servo_of[name]])
    print("DH vs MuJoCo, worst position error over 300 random poses (mm):",
          {k: round(v, 6) for k, v in worst.items()})

    # links
    links = link_table(m)
    with open(OUT / "links_dynamics.csv", "w", newline="", encoding="utf-8") as f:
        w = csv.writer(f)
        w.writerow(["link", "mass (g)", "CoM x,y,z (mm, link frame)", "Ixx,Iyy,Izz principal (g.mm^2)"])
        for r in links:
            w.writerow([r["link"], f"{r['mass_g']:.1f}", r["com_mm"], r["inertia_gmm2"]])

    # joints
    stat = static_gravity(m)
    for k, v in static_floor_cases(m).items():
        if v[0] > stat.get(k, (0, ""))[0]:
            stat[k] = v
    gs = gait_stats(m)
    rows = []
    for j in range(1, m.njnt):
        name = m.joint(j).name
        sv = servo_of[name]
        stall = SERVOS[sv]["stall"]
        rated = stall / 3
        st, st_case = stat.get(name, (0.0, ""))
        g = gs[name]
        need = max(st, g["peak"])
        lo, hi = np.degrees(m.jnt_range[j])
        rows.append(dict(
            joint=name, servo=sv, range=f"{lo:.0f}…{hi:.0f}",
            vmax=(math.pi / 3) / SERVOS[sv]["speed"], vpeak=g["vel"], apeak=g["acc"],
            stall_kgcm=stall / 0.0981, rated_kgcm=rated / 0.0981,
            static_kgcm=st / 0.0981, static_case=st_case,
            peak_kgcm=g["peak"] / 0.0981, rms_kgcm=g["rms"] / 0.0981,
            margin=rated / need if need > 0 else float("inf"),
        ))
    with open(OUT / "joints_kin_dyn.csv", "w", newline="", encoding="utf-8") as f:
        w = csv.writer(f)
        w.writerow(["joint", "servo", "range (deg)", "servo max speed (rad/s)", "sim peak speed (rad/s)",
                    "sim peak accel (rad/s^2)", "stall (kg.cm)", "rated 1/3 (kg.cm)", "static worst (kg.cm)",
                    "static case", "sim peak (kg.cm)", "sim RMS (kg.cm)", "margin rated/need"])
        for r in rows:
            w.writerow([r["joint"], r["servo"], r["range"], f"{r['vmax']:.1f}", f"{r['vpeak']:.2f}",
                        f"{r['apeak']:.0f}", f"{r['stall_kgcm']:.2f}", f"{r['rated_kgcm']:.2f}",
                        f"{r['static_kgcm']:.2f}", r["static_case"], f"{r['peak_kgcm']:.2f}",
                        f"{r['rms_kgcm']:.2f}", f"{r['margin']:.2f}"])
    print(f"wrote {OUT}/dh_table.csv, links_dynamics.csv, joints_kin_dyn.csv")
    print(f"total mass {sum(m.body_mass) * 1000:.0f} g")
    for r in rows:
        print(f"{r['joint']:18s} {r['servo']:8s} need {max(r['static_kgcm'], r['peak_kgcm']):.2f} "
              f"rated {r['rated_kgcm']:.2f} margin {r['margin']:.2f}  vpeak {r['vpeak']:.2f}/{r['vmax']:.1f}")


if __name__ == "__main__":
    main()
