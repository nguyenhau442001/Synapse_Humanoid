# Synapse Humanoid · Mico

![ROS 2](https://img.shields.io/badge/ROS_2-Jazzy-blue?style=flat-square&logo=ros)
![Foxglove](https://img.shields.io/badge/Foxglove-ws%3A8765-purple?style=flat-square)
![Gazebo](https://img.shields.io/badge/Gazebo-Harmonic-brightgreen?style=flat-square)
![License](https://img.shields.io/badge/License-Apache_2.0-lightgrey?style=flat-square)

<p align="center">
  <img src="docs/images/mico.png" width="300" alt="Mico, a 303 mm white and orange humanoid robot with a round head, blue eyes and a Vietnam flag on the chest">
</p>

**Mico** is a 303 mm, 20-DoF bipedal humanoid built from parts sold in Ho Chi Minh City, simulated in Gazebo Harmonic
with ros2_control and viewed in Foxglove. *Cute outside, capable inside. Made in Việt Nam.*

| Height | Mass | DoF | Compute | Sensors | BOM |
| --- | --- | --- | --- | --- | --- |
| 303 mm | 536 g | 20 (legs 2×6, arms 2×3, neck 2) | Radxa Zero 3W (Cortex-A55 + NPU) · STM32F411 (Cortex-M4) | IMU, camera, ToF, 2 mics, 4 foot FSRs | ≈ 5.65M VND |

## TF frames

![TF frames: x forward, y left, z up, with sensor frames and the TF tree](docs/design/sketch_shell_a/images/frames.png)

REP-103 axes (x forward, y left, z up) and REP-120 names. `base_link` sits at the hip point, 115 mm above the floor.
`imu_link` is on the pelvis top plate (z 145 mm) and reads +9.81 m/s² on z at rest. One generator,
[`sim/mini_humanoid/gen_robot.py`](sim/mini_humanoid/gen_robot.py), writes both models with identical frames:

| File | For |
| --- | --- |
| [`sim/mini_humanoid/humanoid.urdf`](sim/mini_humanoid/humanoid.urdf) | ROS, RViz, Foxglove |
| [`src/humanoid_description/urdf/mico.urdf.xacro`](src/humanoid_description/urdf/mico.urdf.xacro) | Gazebo Harmonic: `gz_ros2_control`, IMU, camera, ToF, odometry |

`python sim/mini_humanoid/check_urdf.py` checks that the URDF, the xacro, the controller config and the gait agree.

## Quickstart

Gazebo runs headless in an Ubuntu 24.04 VM (OrbStack on macOS); Foxglove on the Mac connects to it.

```bash
./setup_orbstack.sh                      # on the Mac, once: Ubuntu VM + ROS 2 Jazzy + Gazebo Harmonic + build

# inside the VM (orb -m synapse-humanoid), at the repo root
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install && source install/setup.bash
ros2 launch humanoid_description mico_gz.launch.py gait:=true   # walk 8 steps, turn 60°, wave
```

Foxglove → Open connection → `ws://localhost:8765`, 3D panel with display frame `odom`.
Topics: `/joint_states`, `/imu`, `/odom`, `/tf`, `/camera/image_raw`, `/tof`.

```bash
python3 sim/mini_humanoid/check_gz.py    # with the sim running: fell or not, distance, heading, servo loads
python3 sim/mini_humanoid/kin_dyn.py     # DH tables checked against the URDF, static loads (numpy only)
```

In Gazebo the demo stays up, walks 187 mm and turns 60°. The busiest servos stay below 50% of stall (99th percentile).
Details are in [`sim/mini_humanoid`](sim/mini_humanoid/README.md).

## Repository

| Path | What |
| --- | --- |
| [`sim/mini_humanoid`](sim/mini_humanoid) | Model generator, Gazebo check, DH / dynamics tables, URDF check |
| [`docs/design/sketch_shell_a`](docs/design/sketch_shell_a) | Mico design: shell, skeleton J01–J20, expressions, GA drawing, BOM |
| [`src/humanoid_description`](src/humanoid_description) | URDF / xacro, Gazebo + Foxglove launch, ros2_control config |
| [`src/humanoid_control`](src/humanoid_control) | ros2_control bringup, `mico_gait.py` demo gait |
| [`src/humanoid_perception`](src/humanoid_perception) | Camera pipeline, ONNX Runtime inference |
| [`src/humanoid_infrastructure`](src/humanoid_infrastructure) | Lock-free SPSC queues, thread pool, shared-memory IPC |
| [`src/humanoid_interfaces`](src/humanoid_interfaces) | Custom ROS 2 messages |

## Roadmap

- [x] 20-DoF model, TF frames, DH tables
- [x] Gazebo Harmonic + ros2_control: walk / turn / wave, IMU, camera, ToF, Foxglove
- [x] Mico shell, mechanical skeleton and fit check, BOM at HCMC prices
- [ ] Shell v2.1 from the fit check (arms, head display, hip and heel housings)
- [ ] Foot FSRs as Gazebo contact sensors
- [ ] Footstep planner + closed-form leg IK, then ZMP / LIPM walking
- [ ] STM32 servo firmware and Radxa ROS 2 bringup on the real robot

## License

Apache 2.0, as declared in each package.xml.
