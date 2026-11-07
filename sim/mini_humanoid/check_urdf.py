"""Check that the generated models and the ROS config agree with each other. Needs only the standard library.

    python check_urdf.py

1. humanoid.urdf and mico.urdf.xacro have the same links, joints, origins and limits
2. the xacro declares ros2_control for exactly the 20 revolute joints, plus the IMU, camera and ToF sensors
3. both controllers in mico_controllers.yaml and the gait (mico_gait.JOINTS) use the same 20 joints, same order
4. every sensor / tool frame exists, and camera_optical_frame has the REP-103 optical rotation
"""
import importlib.util
import re
import xml.etree.ElementTree as ET
from pathlib import Path

HERE = Path(__file__).parent
REPO = HERE.parents[1]
XACRO = REPO / "src" / "humanoid_description" / "urdf" / "mico.urdf.xacro"
YAML = REPO / "src" / "humanoid_description" / "config" / "mico_controllers.yaml"
GAIT = REPO / "src" / "humanoid_control" / "scripts" / "mico_gait.py"
FRAMES = ["imu_link", "camera_link", "camera_optical_frame", "tof_link", "l_mic_link", "r_mic_link",
          "l_sole", "r_sole", "l_gripper", "r_gripper"]
SENSORS = {"imu": "imu_link", "head_camera": "camera_link", "tof": "tof_link"}


def joints(root):
    out = {}
    for j in root.iter("joint"):
        if j.find("parent") is None:            # ros2_control <joint> entries
            continue
        o, lim = j.find("origin"), j.find("limit")
        out[j.get("name")] = (j.get("type"), j.find("parent").get("link"), j.find("child").get("link"),
                              o.get("xyz") if o is not None else None, o.get("rpy") if o is not None else None,
                              None if lim is None else (lim.get("lower"), lim.get("upper"), lim.get("effort")))
    return out


def controller_joints(text):
    """{controller: [joints]} from the ros__parameters blocks of the controllers yaml."""
    out, cur, in_joints = {}, None, False
    for line in text.splitlines():
        if re.match(r"^\w", line):
            cur, in_joints = line.rstrip(":"), False
        elif line.strip() == "joints:":
            in_joints = True
            out[cur] = []
        elif in_joints and line.strip().startswith("- "):
            out[cur].append(line.strip()[2:])
        elif in_joints and line.strip():
            in_joints = False
    return out


def main():
    ok = True

    def check(cond, msg):
        nonlocal ok
        print(("ok    " if cond else "FAIL  ") + msg)
        ok &= bool(cond)

    urdf = ET.parse(HERE / "humanoid.urdf").getroot()
    xac = ET.parse(XACRO).getroot()
    ju, jx = joints(urdf), joints(xac)
    revolute = [n for n, j in ju.items() if j[0] == "revolute"]
    check(len(revolute) == 20, f"URDF has {len(revolute)} revolute joints")
    check(ju == jx, "xacro joints (origins, axes, limits) identical to humanoid.urdf")
    lu = {ln.get("name") for ln in urdf.iter("link")}
    check(lu == {ln.get("name") for ln in xac.iter("link")}, f"same {len(lu)} links in URDF and xacro")

    rc = xac.find("ros2_control")
    rc_joints = [j.get("name") for j in rc.findall("joint")]
    check(sorted(rc_joints) == sorted(revolute), "ros2_control covers exactly the 20 revolute joints")
    check(all(j.find("command_interface").get("name") == "position" for j in rc.findall("joint")),
          "every joint takes a position command")
    refs = {g.get("reference"): g for g in xac.iter("gazebo") if g.get("reference")}
    for sensor, link in SENSORS.items():
        g = refs.get(link)
        check(g is not None and g.find(f"sensor[@name='{sensor}']") is not None,
              f"Gazebo sensor '{sensor}' on {link}")

    ctrl = controller_joints(YAML.read_text(encoding="utf-8"))
    spec = importlib.util.spec_from_file_location("mico_gait", GAIT)
    gait = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(gait)
    check(sorted(gait.JOINTS) == sorted(revolute), "mico_gait.JOINTS are the 20 URDF joints")
    for name, js in ctrl.items():
        check(js == gait.JOINTS, f"{name} joints match mico_gait.JOINTS (same order)")

    for f in FRAMES:
        check(f in lu, f"frame {f}")
    opt = ju.get("camera_optical_joint")
    check(opt is not None and [round(float(v), 4) for v in opt[4].split()] == [-1.5708, 0.0, -1.5708],
          "camera_optical_frame rpy = (-pi/2, 0, -pi/2): z forward, x right, y down")

    print("\nall checks passed" if ok else "\nSOME CHECKS FAILED")
    raise SystemExit(0 if ok else 1)


if __name__ == "__main__":
    main()
