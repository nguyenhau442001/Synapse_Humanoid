#!/usr/bin/env python3
"""Headless pass/fail check of the Mico demo in Gazebo: did it stay up, how far did it walk, how hard were the servos.

Start the simulation with the gait first, then run this in a second shell:
    ros2 launch humanoid_description mico_gz.launch.py camera:=false foxglove:=false gait:=true
    python3 sim/mini_humanoid/check_gz.py            # waits for the whole sequence (~38 s of sim time)

Reads /odom (ground-truth base pose), /imu and /joint_states (effort), and compares each joint's peak and RMS
effort with its servo's stall torque (the URDF <limit effort>). Continuous rating is taken as 1/3 of stall.
The first --settle seconds (spawn drop onto the floor) are ignored; "p99" is the 99th-percentile effort.
Writes tables/gz_servo_loads.csv.
"""
import argparse
import csv
import math
import os
import xml.etree.ElementTree as ET

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import Imu, JointState

HERE = os.path.dirname(os.path.abspath(__file__))


def stall_torques(urdf):
    root = ET.parse(urdf).getroot()
    return {j.get("name"): float(j.find("limit").get("effort"))
            for j in root.findall("joint") if j.get("type") == "revolute"}


def tilt_deg(q):
    """Angle between the body z axis and world z, from a quaternion."""
    x, y = q.x, q.y
    return math.degrees(math.acos(max(-1.0, min(1.0, 1 - 2 * (x * x + y * y)))))


def yaw_deg(q):
    return math.degrees(math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z)))


class Check(Node):
    def __init__(self, stall, settle):
        super().__init__("mico_check")
        self.stall = stall
        self.settle = settle
        self.t0 = None
        self.now = None
        self.start = None
        self.last = None
        self.z_min = 1e9
        self.tilt_max = 0.0
        self.t_fall = None
        self.eff = {j: [] for j in stall}
        self.acc = []
        self.create_subscription(Odometry, "/odom", self.on_odom, 50)
        self.create_subscription(Imu, "/imu", self.on_imu, 50)
        self.create_subscription(JointState, "/joint_states", self.on_js, 50)

    def stamp(self, msg):
        return msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9

    def settled(self, msg):
        t = self.stamp(msg)
        if self.t0 is None:
            self.t0 = t
        self.now = t
        return t - self.t0 >= self.settle

    def on_odom(self, m):
        if not self.settled(m):
            return
        p, q = m.pose.pose.position, m.pose.pose.orientation
        if self.start is None:
            self.start = (p.x, p.y, yaw_deg(q))
        self.last = (p.x, p.y, p.z, yaw_deg(q))
        self.z_min = min(self.z_min, p.z)
        self.tilt_max = max(self.tilt_max, tilt_deg(q))
        if self.t_fall is None and tilt_deg(q) > 30:
            self.t_fall = self.now - self.t0

    def on_imu(self, m):
        if self.settled(m) and len(self.acc) < 200:
            self.acc.append(m.linear_acceleration.z)

    def on_js(self, m):
        if not self.settled(m):
            return
        for name, e in zip(m.name, m.effort):
            if name in self.eff:
                self.eff[name].append(abs(e))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--duration", type=float, default=38.0, help="sim seconds to record")
    ap.add_argument("--settle", type=float, default=3.0, help="sim seconds to skip after the first message")
    ap.add_argument("--urdf", default=os.path.join(HERE, "humanoid.urdf"))
    a = ap.parse_args()

    rclpy.init()
    node = Check(stall_torques(a.urdf), a.settle)
    while rclpy.ok():
        rclpy.spin_once(node, timeout_sec=0.5)
        if node.t0 is not None and node.now is not None and node.now - node.t0 >= a.duration:
            break
    n = len(next(iter(node.eff.values())))
    rate = n / max(node.now - node.t0 - a.settle, 1e-9)

    x0, y0, yaw0 = node.start
    x, y, z, yaw = node.last
    fell = node.z_min < 0.07 or node.tilt_max > 45
    print(f"sim time {node.now - node.t0:.1f} s, /joint_states {rate:.0f} Hz")
    print(f"IMU z while standing: {sum(node.acc) / max(len(node.acc), 1):+.2f} m/s^2")
    if node.t_fall is not None:
        print(f"tilt passed 30 deg at t = {node.t_fall:.1f} s (gait sequence starts ~2 s after the first message)")
    print(f"{'FELL' if fell else 'STAYED UP'}: base z min {node.z_min * 1000:.0f} mm, max tilt {node.tilt_max:.1f} deg")
    print(f"moved {math.hypot(x - x0, y - y0) * 1000:.0f} mm (dx {(x - x0) * 1000:+.0f}, dy {(y - y0) * 1000:+.0f}), "
          f"heading {yaw - yaw0:+.1f} deg")

    rows = []
    for j, s in node.stall.items():
        e = sorted(node.eff[j]) or [0.0]
        p99 = e[int(0.99 * (len(e) - 1))]
        rms = math.sqrt(sum(v * v for v in e) / len(e))
        over = sum(v > s / 3 for v in e) / rate
        rows.append([j, s, p99, rms, 100 * p99 / s, over])
    rows.sort(key=lambda r: -r[4])
    print(f"\n{'joint':18s}{'stall N·m':>10s}{'p99':>8s}{'rms':>8s}{'p99 %':>8s}{'s > cont':>9s}")
    for r in rows[:8]:
        print(f"{r[0]:18s}{r[1]:10.3f}{r[2]:8.3f}{r[3]:8.3f}{r[4]:7.0f}%{r[5]:9.2f}")
    os.makedirs(os.path.join(HERE, "tables"), exist_ok=True)
    with open(os.path.join(HERE, "tables", "gz_servo_loads.csv"), "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["joint", "stall_Nm", "p99_Nm", "rms_Nm", "p99_pct_stall", "s_above_continuous"])
        for r in sorted(rows):
            w.writerow([r[0]] + [f"{v:.4f}" for v in r[1:]])
    node.destroy_node()
    rclpy.shutdown()
    raise SystemExit(1 if fell else 0)


if __name__ == "__main__":
    main()
