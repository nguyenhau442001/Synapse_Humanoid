# Synapse Humanoid 🤖

![ROS 2](https://img.shields.io/badge/ROS_2-Jazzy-blue?style=flat-square&logo=ros)
![C++](https://img.shields.io/badge/C++-17%20/%2020-00599C?style=flat-square&logo=c%2B%2B)
![ONNX Runtime](https://img.shields.io/badge/ONNX_Runtime-1.18+-005CED?style=flat-square)
![ros2_control](https://img.shields.io/badge/ros2__control-Jazzy-green?style=flat-square)
![License](https://img.shields.io/badge/License-Apache_2.0-lightgrey?style=flat-square)

A modular, high-performance software stack for humanoid robotics, bipedal locomotion, and real-time AI perception built on **ROS 2 Jazzy**.

---

## 🏛️ System Architecture

```text
                               +-----------------------------------+
                               |     Head RGB-D Camera Sensor      |
                               +-----------------------------------+
                                                 | (sensor_msgs/Image)
                                                 v
+-----------------------------------------------------------------------------------+
| humanoid_perception                                                               |
|  - Camera Subscriber & Message Synchronizer (RGB + Depth)                         |
|  - High-Throughput Zero-Copy Ingestion via SPSC Lock-Free Ring Buffer             |
|  - ONNX Runtime / TensorRT Multi-Threaded Inference Engine                        |
|  - 3D Spatial Bounding Box & Coordinate Estimator                                 |
+-----------------------------------------------------------------------------------+
                                                 | (humanoid_interfaces/Detection3DArray)
                                                 v
+-----------------------------------------------------------------------------------+
| humanoid_infrastructure                                                           |
|  - Lock-Free Concurrent Ring Buffers (SPSC Queue)                                 |
|  - Thread Pool with Core Pinning & Priority Worker Dispatch                       |
|  - POSIX Shared Memory (SHM) Zero-Copy Coprocessor IPC                            |
+-----------------------------------------------------------------------------------+
                                                 |
                                                 v
+-----------------------------------------------------------------------------------+
| humanoid_control (ros2_control)                                                   |
|  - Controller Manager (1 kHz RT Loop)                                             |
|  - Lower Body Bipedal Controller (12-DOF Legs: Hip Yaw/Roll/Pitch, Knee, Ankle)   |
|  - Upper Body Controller (Torso, Neck, Head, 6-DOF Dual Arms)                     |
|  - Real-Time Joint Limit & Velocity Safety Guards                                 |
+-----------------------------------------------------------------------------------+
                                                 |
                                                 v
+-----------------------------------------------------------------------------------+
| humanoid_description                                                              |
|  - Parametric Xacro Kinematic Model (20+ Active Degrees of Freedom)               |
|  - Hardware Abstraction Interface (ros2_control tags)                             |
|  - RViz2 Visualization & Interactive Joint State Controls                         |
+-----------------------------------------------------------------------------------+
```

---

## 📦 Packages Overview

All packages are organized under `src/` following modern ROS 2 workspace standards:

| Package | Language | Description |
|---|---|---|
| [`humanoid_perception`](src/humanoid_perception) | C++17 | Camera subscriber nodes, image synchronizers, and ONNX Runtime / TensorRT deep learning inference pipelines. |
| [`humanoid_control`](src/humanoid_control) | C++17 / YAML | `ros2_control` configuration files, real-time joint trajectory bridges, and controller manager bringup. |
| [`humanoid_infrastructure`](src/humanoid_infrastructure) | C++17 | Core low-latency infrastructure: lock-free SPSC queues, worker thread pools, and shared-memory (SHM) IPC. |
| [`humanoid_description`](src/humanoid_description) | URDF / Xacro | Kinematic model of the bipedal humanoid (pelvis, torso, 6-DOF legs, arms, head), visual/collision meshes, and RViz display launches. |
| [`humanoid_interfaces`](src/humanoid_interfaces) | ROS IDL | Custom ROS 2 message definitions (`Detection3D`, `Detection3DArray`, `BipedalState`). |

---

## ⚙️ Kinematic Hierarchy & Degrees of Freedom (DOF)

The Synapse Humanoid model specifies 20+ active actuated joints:

- **Torso & Head (3 DOF):**
  - Torso Yaw: `[-1.57, +1.57] rad`
  - Head Yaw: `[-1.2, +1.2] rad`
  - Head Pitch: `[-0.8, +0.8] rad`
- **Bipedal Lower Body (12 DOF, 6 DOF per leg):**
  - Hip Yaw, Hip Roll, Hip Pitch
  - Knee Pitch: `[0.0, +2.2] rad`
  - Ankle Pitch, Ankle Roll
- **Upper Body Dual Arms (6 DOF, 3 DOF per arm):**
  - Shoulder Pitch, Shoulder Roll
  - Elbow Pitch

---

## 🚀 Quickstart & Development Setup

We provide streamlined automated setup scripts tailored for both **macOS (via OrbStack)** and **Native Ubuntu 24.04 LTS**.

### 🍎 Option A: macOS via OrbStack (Recommended)
OrbStack provides an ultra-fast, lightweight Ubuntu 24.04 environment on Apple Silicon without the heavy overhead and battery drain of traditional VMs (like UTM). Your macOS workspace directory is mounted with native SSD I/O speeds.

1. **Run the Automated Setup Script on macOS:**
   ```bash
   chmod +x setup_orbstack.sh
   ./setup_orbstack.sh
   ```
   *This automatically installs OrbStack, launches a clean `ubuntu:noble` instance named `synapse-humanoid`, provisions ROS 2 Jazzy + dependencies, and performs an initial verification build.*

2. **Daily Development Workflow:**
   - **Enter Linux Terminal:**
     ```bash
     orb -m synapse-humanoid
     ```
   - **Or build directly from your macOS terminal:**
     ```bash
     orb -m synapse-humanoid bash -c "source /opt/ros/jazzy/setup.bash && cd \$PWD && colcon build --symlink-install"
     ```

---

### 🐧 Option B: Native Ubuntu 24.04 LTS (Robot IPC / Workstation)

1. **Run the Automated Setup Script on Ubuntu:**
   ```bash
   chmod +x setup_ubuntu.sh
   ./setup_ubuntu.sh
   ```

2. **Build Workspace:**
   ```bash
   source /opt/ros/jazzy/setup.bash
   colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
   source install/setup.bash
   ```

---

### 🎯 Bringup Commands

#### 1. High-Performance 3D Visualization via Foxglove Studio (Recommended for macOS)
Enjoy native Apple Silicon GPU Metal rendering (60–120 FPS ProMotion, zero VM lag, ultra-low battery drain):
```bash
# Inside OrbStack Linux:
ros2 launch humanoid_description foxglove.launch.py
```
Then on macOS: Open **Foxglove Studio** -> Click **Open connection** -> Enter `ws://localhost:8765` -> Click **Connect**.

#### 2. Launch Robot Visualization in RViz2 (Standard X11)
Inspect the humanoid kinematic model and interact with joint limit sliders in RViz2:
```bash
ros2 launch humanoid_description display.launch.py
```

#### 2. Launch Real-Time Perception Pipeline
Launch the RGB-D camera subscriber and ONNX Runtime / TensorRT inference pipeline:
```bash
ros2 launch humanoid_perception perception.launch.py
```

#### 3. Launch Joint Trajectory Controllers (`ros2_control`)
Bring up the controller manager and joint trajectory bridges:
```bash
ros2 launch humanoid_control humanoid_control.launch.py
```

---

## 🗺️ Engineering Roadmap

- [x] **Phase 1: Architecture Migration & Scaffolding**
  - [x] Modular workspace reorganization under `src/`.
  - [x] Implementation of lock-free queues and zero-copy shared memory IPC primitives.
  - [x] Full-body bipedal humanoid kinematic definition with `ros2_control` interfaces.
- [ ] **Phase 2: Bipedal Locomotion & Whole-Body Control (WBC)**
  - [ ] Linear Inverted Pendulum Model (LIPM) walking pattern generator.
  - [ ] Quadratic Programming (QP) Whole-Body Controller with contact force constraints.
  - [ ] MuJoCo / Isaac Sim simulation environment integration.
- [ ] **Phase 3: Visual-Spatial AI Foundation Integration**
  - [ ] YOLO-World / Grounding DINO integration for zero-shot object manipulation.
  - [ ] Head-mounted point cloud fusion with OctoMap / VDB-Fusion for 3D obstacle avoidance.

---

## 📄 License
This repository is licensed under the [Apache 2.0 License](LICENSE).
