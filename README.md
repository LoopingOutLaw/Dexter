<div align="center">

```
██████╗ ███████╗██╗  ██╗████████╗███████╗██████╗
██╔══██╗██╔════╝╚██╗██╔╝╚══██╔══╝██╔════╝██╔══██╗
██║  ██║█████╗   ╚███╔╝    ██║   █████╗  ██████╔╝
██║  ██║██╔══╝   ██╔██╗    ██║   ██╔══╝  ██╔══██╗
██████╔╝███████╗██╔╝ ██╗   ██║   ███████╗██║  ██║
╚═════╝ ╚══════╝╚═╝  ╚═╝   ╚═╝   ╚══════╝╚═╝  ╚═╝
```

### 🦾 3-DOF Robotic Arm — ROS 2 · MoveIt 2 · Gazebo · Arduino

[![ROS 2](https://img.shields.io/badge/ROS_2-Humble_|_Iron_|_Jazzy-22314E?logo=ros&logoColor=white)](https://docs.ros.org)
[![MoveIt 2](https://img.shields.io/badge/MoveIt_2-motion_planning-orange)](https://moveit.ros.org)
[![License](https://img.shields.io/badge/License-Apache_2.0-blue)](LICENSE)
[![Platform](https://img.shields.io/badge/Platform-Linux-lightgrey?logo=linux)](https://ubuntu.com)

*Simulate it. Deploy it. Control it from your browser.*

</div>

---

A full-stack ROS 2 project for a 3-DOF robotic arm with a parallel gripper, supporting both **Gazebo simulation** and **real hardware** control via Arduino. Includes MoveIt 2 motion planning, a ros2_control hardware interface, and a browser-based web control panel with voice command support.

---

## Table of Contents

- [Overview](#overview)
- [Architecture](#architecture)
- [Package Structure](#package-structure)
- [Prerequisites](#prerequisites)
- [Installation](#installation)
- [Usage](#usage)
- [Package Details](#package-details)
- [Hardware Setup](#hardware-setup)
- [Web Interface](#web-interface)

---

## Overview

Dexter is a 3-DOF serial robotic arm with a parallel gripper (joint 4/5), controlled via four servo motors connected to an Arduino. The ROS 2 stack provides:

- **URDF/Xacro** robot description with Gazebo and ros2_control plugins
- **ros2_control** hardware interface over serial (LibSerial) for real robot control
- **MoveIt 2** for motion planning (OMPL, Pilz) with an RViz panel
- **Task action server** (C++ and Python variants) for pre-defined task execution
- **Web UI** with button and voice command control, served by Flask

| Feature | Simulation | Real Robot |
|---|---|---|
| Gazebo (Ignition / Gz Sim) | ✅ | — |
| ros2_control | ✅ | ✅ |
| MoveIt 2 | ✅ | ✅ |
| Web Interface | ✅ | ✅ |
| Arduino Firmware | — | ✅ |

---

## Gallery

### 🖼️ Robot Model & Simulation

<div align="center">

| Real Robot | Gazebo Simulation |
|:---:|:---:|
| ![Real Robot](docs/images/real_robot.jpg) | ![Gazebo Simulation](docs/images/gazebo_sim.jpg) |

| RViz — MoveIt 2 Planning | Web Control Panel |
|:---:|:---:|
| ![MoveIt RViz](docs/images/moveit_rviz.jpg) | ![Web UI](docs/images/web_ui.jpg) |

</div>

> 💡 **To add images:** drop your files into `docs/images/` and the table above will render them automatically.

---

### 🎬 Demo Videos

<div align="center">

**Gazebo Simulation Demo**

[![Gazebo Demo](https://img.shields.io/badge/▶_Watch-Simulation_Demo-red?logo=youtube&logoColor=white&style=for-the-badge)](https://youtube.com/your-link-here)

---

**Real Robot Pick & Place**

[![Real Robot Demo](https://img.shields.io/badge/▶_Watch-Real_Robot_Demo-red?logo=youtube&logoColor=white&style=for-the-badge)](https://youtube.com/your-link-here)

</div>

---

## Architecture

```
┌─────────────────────────────────────────────────────────┐
│                     dexter_bringup                      │
│          simulated_robot.launch / real_robot.launch     │
└────────────┬────────────────┬────────────────┬──────────┘
             │                │                │
     ┌───────▼──────┐ ┌───────▼──────┐ ┌──────▼──────────┐
     │dexter_descrip│ │dexter_control│ │  dexter_moveit  │
     │  URDF/Xacro  │ │ros2_control  │ │  MoveIt 2 /     │
     │  Gazebo      │ │  interface   │ │  move_group     │
     └──────────────┘ └───────┬──────┘ └──────┬──────────┘
                              │               │
                       ┌──────▼───────────────▼──────┐
                       │        dexter_remote         │
                       │  task_server (C++ / Python)  │
                       │  web_interface (Flask)        │
                       └──────────────────────────────┘
                                     │
                              Browser / Voice
```

On the real robot, `dexter_controller` communicates with the Arduino over serial at 115200 baud using a compact ASCII protocol (`b090,s090,e090,g000,`).

---

## Package Structure

```
.
├── dexter_bringup/          # Top-level launch files
├── dexter_controller/       # ros2_control HW interface + slider control node
│   ├── src/
│   │   ├── dexter_interface.cpp   # SystemInterface (LibSerial → Arduino)
│   │   └── slider_control.cpp     # JointState → JointTrajectory bridge
│   └── config/
│       └── dexter_controllers.yaml
├── dexter_description/      # URDF, meshes, Gazebo launch, RViz config
│   └── urdf/
│       ├── dexter.urdf.xacro
│       ├── dexter_gazebo.xacro
│       └── dexter_ros2_control.xacro
├── dexter_firmware/         # Arduino sketches + serial test nodes
│   └── firmware/
│       ├── robot_control/         # Main servo control sketch
│       └── motor_calibration/
├── dexter_moveit/           # MoveIt 2 config (SRDF, kinematics, planners)
├── dexter_msgs/             # Custom interfaces (DexterTask action, services)
├── dexter_remote/           # Task action server + Flask web interface
│   ├── src/task_server.cpp        # C++ MoveGroupInterface task server
│   ├── dexter_remote/
│   │   ├── task_server.py         # Python MoveItPy task server
│   │   └── web_interface.py       # Flask + ROS 2 bridge
│   └── templates/index.html
└── dexter_utils/            # Euler ↔ Quaternion conversion services
```

---

## Prerequisites

| Requirement | Version |
|---|---|
| ROS 2 | Humble **or** Iron / Jazzy |
| MoveIt 2 | Matching ROS 2 distro |
| Gazebo | Ignition (Humble) / Gz Sim (Iron+) |
| libserial-dev | any |
| python3-flask | any |
| Arduino IDE | For firmware upload |

---

## Installation

**1. Clone into your ROS 2 workspace:**

```bash
cd ~/ros2_ws/src
git clone <repo-url> dexter
```

**2. Install dependencies:**

```bash
cd ~/ros2_ws
rosdep install --from-paths src --ignore-src -r -y
```

**3. Build:**

```bash
colcon build --symlink-install
source install/setup.bash
```

**4. (Real robot only) Upload firmware:**

Open `dexter_firmware/firmware/robot_control/robot_control.ino` in the Arduino IDE and upload to your Arduino board. Confirm the serial port (default `/dev/ttyACM0`).

---

## Usage

### Simulated Robot

Launch Gazebo, ros2_control, MoveIt 2, and the web interface in one command:

```bash
ros2 launch dexter_bringup simulated_robot.launch.py
```

### Real Robot

```bash
ros2 launch dexter_bringup real_robot.launch.py
```

### Individual Components

| Launch File | Description |
|---|---|
| `dexter_description/launch/display.launch.py` | RViz model viewer with joint sliders |
| `dexter_description/launch/gazebo.launch.py` | Gazebo only |
| `dexter_controller/launch/controller.launch.py is_sim:=True` | ros2_control + spawners |
| `dexter_controller/launch/slider_controller.launch.py` | Joint slider → controller bridge |
| `dexter_moveit/launch/moveit.launch.py` | MoveIt 2 + RViz |
| `dexter_remote/launch/remote_interface.launch.py` | Task server + web interface |

**Use the Python task server instead of C++:**

```bash
ros2 launch dexter_remote remote_interface.launch.py use_python:=True
```

### Sending Tasks Manually

```bash
ros2 action send_goal /task_server dexter_msgs/action/DexterTask "{task_number: 0}"
# 0 = Home/Wake   1 = Pick   2 = Sleep
```

### Angle Conversion Utilities

```bash
# Start the conversion service
ros2 run dexter_utils angle_conversion

# Call from CLI
ros2 service call /euler_to_quaternion dexter_msgs/srv/EulerToQuaternion "{roll: 0.0, pitch: 1.57, yaw: 0.0}"
```

---

## Package Details

### dexter_controller

The `DexterInterface` class implements `hardware_interface::SystemInterface`. On `write()`, joint positions (radians) are converted to servo angles (degrees) and serialized to the Arduino using the protocol:

```
b<angle>,s<angle>,e<angle>,g<angle>,
```

where each angle is zero-padded to three digits (e.g., `b090,s135,e045,g000,`).

The `slider_control` node bridges `/joint_commands` (`sensor_msgs/JointState`) to the `arm_controller` and `gripper_controller` trajectory topics, enabling live control from `joint_state_publisher_gui`.

### dexter_moveit

Configured with KDL kinematics (position-only IK) and OMPL + Pilz planners. The SRDF defines two planning groups: `arm` (joints 1–3) and `gripper` (joints 4–5). Joint 5 mimics joint 4 with a −1 multiplier.

### dexter_remote

The **task server** exposes a `DexterTask` action server. Goal `task_number` maps to pre-defined joint goals:

| Task | Arm (j1, j2, j3) | Gripper (j4, j5) |
|---|---|---|
| 0 — Home/Wake | `[0.0, 0.0, 0.0]` | `[-0.7, 0.7]` |
| 1 — Pick | `[-1.14, -0.6, -0.07]` | `[0.0, 0.0]` |
| 2 — Sleep | `[-1.57, 0.0, -0.9]` | `[0.0, 0.0]` |

The **web interface** runs Flask on `http://0.0.0.0:5000` and uses callbacks + `threading.Event` to bridge HTTP requests to the ROS 2 action server without blocking the executor.

---

## Hardware Setup

```
Arduino Pins
├── D8  → Base servo
├── D9  → Shoulder servo
├── D10 → Elbow servo
└── D11 → Gripper servo

USB Serial: /dev/ttyACM0  (115200 baud)
```

To use a different port, edit `dexter_description/urdf/dexter_ros2_control.xacro`:

```xml
<param name="port">/dev/ttyACM0</param>
```

---

## Web Interface

Once the bringup is running, open a browser and navigate to:

```
http://localhost:5000
```

The panel provides:

- **Task buttons** — Home, Pick, Sleep, Wake
- **Voice control** — Web Speech API (Chrome/Edge); speak "home", "pick", "sleep", "wake", "grab", "rest", etc.
- **Activity log** — real-time feedback from the action server
- **Health endpoint** — `GET /health` returns task server connection status

---
