"""Check humanoid.urdf against humanoid.xml.

pip install mujoco numpy urdf-parser-py yourdfpy
python check_urdf.py

1. parse with urdf_parser_py (the Python twin of urdfdom used by robot_state_publisher)
2. forward kinematics of every joint origin and sensor frame, URDF (yourdfpy) vs MJCF (MuJoCo), random poses
3. camera_optical_frame axes (z forward, x right, y down)
4. MuJoCo loads the URDF directly
"""
from pathlib import Path

import mujoco
import numpy as np
import yourdfpy
from urdf_parser_py.urdf import URDF

HERE = Path(__file__).parent
FRAMES = ["imu_link", "camera_link", "tof_link", "l_mic_link", "r_mic_link",
          "l_sole", "r_sole", "l_gripper", "r_gripper"]


def main():
    urdf = str(HERE / "humanoid.urdf")
    r = URDF.from_xml_file(urdf)
    movable = [j for j in r.joints if j.type == "revolute"]
    fixed = [j for j in r.joints if j.type == "fixed"]
    print(f"urdf_parser_py : root={r.get_root()}, links={len(r.links)}, revolute={len(movable)}, fixed={len(fixed)}")

    y = yourdfpy.URDF.load(urdf, build_scene_graph=True, load_meshes=False)
    m = mujoco.MjModel.from_xml_path(str(HERE / "humanoid.xml"))
    d = mujoco.MjData(m)
    rng = np.random.default_rng(0)
    worst_p = worst_r = 0.0
    for _ in range(200):
        q = {j.name: rng.uniform(j.limit.lower, j.limit.upper) for j in movable}
        y.update_cfg(q)
        d.qpos[:] = 0
        d.qpos[3] = 1
        for n, v in q.items():
            d.qpos[m.joint(n).qposadr[0]] = v
        mujoco.mj_kinematics(m, d)
        base = d.xpos[m.body("base").id]
        for j in movable:
            T = y.get_transform(j.child, "base_link")
            worst_p = max(worst_p, np.abs(T[:3, 3] - (d.xanchor[m.joint(j.name).id] - base)).max())
        for f in FRAMES:
            T = y.get_transform(f, "base_link")
            s = m.site(f).id
            worst_p = max(worst_p, np.abs(T[:3, 3] - (d.site_xpos[s] - base)).max())
            worst_r = max(worst_r, np.abs(T[:3, :3] - d.site_xmat[s].reshape(3, 3)).max())
    print(f"FK URDF vs MJCF: worst position error {worst_p * 1000:.4f} mm, rotation error {worst_r:.1e} "
          f"(200 random poses, {len(movable)} joints + {len(FRAMES)} sensor frames)")

    y.update_cfg({j.name: 0.0 for j in movable})
    R = y.get_transform("camera_optical_frame", "base_link")[:3, :3]
    print("camera optical : x", R[:, 0].round(3), "y", R[:, 1].round(3), "z", R[:, 2].round(3))

    mu = mujoco.MjModel.from_xml_path(urdf)
    print(f"MuJoCo import  : {mu.njnt} joints, {mu.body_subtreemass[0] * 1000:.0f} g "
          f"(MJCF {m.body_subtreemass[0] * 1000:.0f} g)")


if __name__ == "__main__":
    main()
