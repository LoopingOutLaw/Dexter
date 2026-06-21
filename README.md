<div align="center">

```
 ██████╗ ███████╗██╗  ██╗████████╗███████╗██████╗
 ██╔══██╗██╔════╝╚██╗██╔╝╚══██╔══╝██╔════╝██╔══██╗
 ██║  ██║█████╗   ╚███╔╝    ██║   █████╗  ██████╔╝
 ██║  ██║██╔══╝   ██╔██╗    ██║   ██╔══╝  ██╔══██╗
 ██████╔╝███████╗██╔╝ ██╗   ██║   ███████╗██║  ██║
 ╚═════╝ ╚══════╝╚═╝  �═╝   ╚═╝   ╚══════╝╚═╝  ╚═╝
```

### 🦾 3-DOF Robotic Arm — ROS 2 · MoveIt 2 · Gazebo · Arduino

[![ROS 2](https://img.shields.io/badge/ROS_2-Humble_|_Iron_|_Jazzy-22314E?logo=ros&logoColor=white)](https://docs.ros.org)
[![MoveIt 2](https://img.shields.io/badge/MoveIt_2-motion_planning-orange)](https://moveit.ros.org)
[![License](https://img.shields.io/badge/License-Apache_2.0-blue)](LICENSE)
[![Platform](https://img.shields.io/badge/Platform-Linux-lightgrey?logo=linux)](https://ubuntu.com)

*Simulate it. Deploy it. Control it from your browser — or a glove.*

</div>

---

A full-stack ROS 2 project for a 3-DOF robotic arm with a parallel gripper (joint 4/5), supporting both **Gazebo simulation** and **real hardware** control via Arduino. Includes MoveIt 2 motion planning, a ros2_control hardware interface, a browser-based web control panel with voice command support, **and a gesture control system** driven by an ESP32 glove with an IMU sensor.

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

Dexter is a 3-DOF serial robotic arm with a parallel gripper (joint 4/5), controlled via four servo motors connected to an Arduino. 

A **gesture control system** can also drive the arm via an ESP32-based glove. The glove features an MPU6050 IMU and a flex sensor, streaming orientation data over WiFi (UDP) to the ROS 2 PC, where it is processed into smooth, jerk-bounded joint trajectories.

The ROS 2 stack provides:

- **URDF/Xacro** robot description with Gazebo and ros2_control plugins
- **ros2_control** hardware interface over serial (LibSerial) for real robot control
- **MoveIt 2** for motion planning (OMPL, Pilz) with an RViz panel
- **Task action server** (C++ and Python variants) for pre-defined task execution
- **Web UI** with button and voice command control, served by Flask
- **Gesture control** — IMU-driven, with a 4-stage smoothing pipeline

| Feature | Simulation | Real Robot |
|---|---|---|
| Gazebo (Ignition / Gz Sim) | ✅ | — |
| ros2_control | ✅ | ✅ |
| MoveIt 2 | ✅ | ✅ |
| Web Interface | ✅ | ✅ |
| Arduino Firmware | — | ✅ |
| Gesture Glove | — | ✅ |

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

### Core Robot Stack

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

### Gesture Control System

```
┌──────────────┐      WiFi (UDP)       ┌─────────────────────────────┐
│  ESP32 Glove │  ───────────────────► │  dexter_gesture (ROS 2)    │
│ MPU6050 IMU  │   roll,pitch,yaw,flex │  gesture_receiver node       │
│ Flex sensor  │   50 Hz, port 5005   │                             │
└──────────────┘                      │  4-stage smoothing pipeline │
                                      │    1. Deadband              │
                                      │    2. EMA low-pass          │
                                      │    3. Jerk-bounded S-curve  │
                                      │    4. Publish deadband      │
                                      └─────────────┬───────────────┘
                                                    │
                              ┌─────────────────────┼───────────────────┐
                              │                     │                   │
                    arm_controller         gripper_controller      (diagnostics)
                    joint_trajectory       joint_trajectory
                              │                     │
                         ┌────▼────┐           ┌────▼────┐
                         │  Real   │           │  Real   │
                         │  Robot  │           │ Gripper │
                         └─────────┘           └─────────┘
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
├── dexter_firmware/         # Arduino and ESP32 sketches
│   └── firmware/
│       ├── robot_control/         # Main servo control sketch (Arduino Uno)
│       ├── motor_calibration/
│       └── gesture_esp32/         # Glove firmware (ESP32 DevKit)
├── dexter_gesture/          # ESP32 UDP → arm, 4-stage jerk-bounded smoother
│   ├── dexter_gesture/
│   │   ├── gesture_receiver.py   # ROS 2 node (4-stage smoothing pipeline)
│   │   ├── motion_smoother.py    # JointSmoother (jerk-bounded profile)
│   │   └── udp_test_sender.py    # PC-side simulator
│   ├── launch/                   # gesture_control, gesture_sim
│   └── config/gesture_config.yaml
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
| ESP32 Board Support | For glove firmware (see dexter_firmware/README.md) |

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

For the gesture glove, flash `dexter_firmware/firmware/gesture_esp32/gesture_esp32.ino` to the ESP32 — see `dexter_firmware/README.md` for the full procedure.

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

### Gesture Control

Start the gesture node (real controllers must be up):

```bash
ros2 launch dexter_gesture gesture_control.launch.py
```

Then flash and power on the ESP32 glove.

To test without the glove, run the UDP simulator:

```bash
python3 install/dexter_gesture/lib/python3.12/site-packages/dexter_gesture/udp_test_sender.py
```

See `dexter_gesture/README.md` for tuning, troubleshooting, and simulator modes.

### Individual Components

| Launch File | Description |
|---|---|
| `dexter_description/launch/display.launch.py` | RViz model viewer with joint sliders |
| `dexter_description/launch/gazebo.launch.py` | Gazebo only |
| `dexter_controller/launch/controller.launch.py is_sim:=True` | ros2_control + spawners |
| `dexter_controller/launch/slider_controller.launch.py` | Joint slider → controller bridge |
| `dexter_moveit/launch/moveit.launch.py` | MoveIt 2 + RViz |
| `dexter_remote/launch/remote_interface.launch.py` | Task server + web interface |
| `dexter_gesture/launch/gesture_control.launch.py` | Gesture receiver node |
| `dexter_gesture/launch/gesture_sim.launch.py` | Gazebo + gesture stack (no robot needed) |

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

### dexter_gesture

The `gesture_receiver` node consumes UDP orientation packets from the ESP32 glove (`"roll,pitch,yaw,flex\n"`) and publishes smooth `JointTrajectory` commands to both controllers. The 4-stage pipeline runs:

1. **Deadband** — drops sensor jitter below `deadband_deg`.
2. **EMA low-pass** with `low_pass_alpha=0.12` smooths the gesture.
3. **Jerk-bounded S-curve** planner on a 100 Hz wall-clock timer, **decoupled from UDP rate** so a 200 ms packet hiccup never makes the arm hop. Caps: `max_velocity_rad_s=1.2`, `max_acceleration_rad_s2=6.0`, `max_jerk_rad_s3=8.0`.
4. **Publish deadband** at 0.005 rad cuts idle chatter on the trajectory topics.

The gripper uses a separate, softer profile (v/a/j = 0.8 / 4.0 / 5.0). Trajectory topics are `/arm_controller/joint_trajectory` (joints 1–3) and `/gripper_controller/joint_trajectory` (joint_4).

Calibration: hand pose at BOOT becomes (0, 0, 0). Flex ADC values are mapped to gripper angle with `flex_open_adc` / `flex_closed_adc`. Wrist twist drives joint_1 directly; pitch is split with `shoulder_gain=1.0` / `elbow_gain=0.6`; roll adds a small × 0.4 offset to joint_3.

All 21 knobs are launch-time arguments — see `dexter_gesture/README.md` for the recipes.

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

### Robot Arm (Arduino)

**Servo wiring:**

```
Arduino Uno
├── D8  → Base servo     (signal wire; VCC/GND from separate 5 V supply)
├── D9  → Shoulder servo
├── D10 → Elbow servo
└── D11 → Gripper servo

USB Serial: /dev/ttyACM0  (115200 baud)
```

> ⚠️ **Power:** Servos draw more current than the Arduino 5V pin can safely supply. Use an external 5V supply (e.g., a BEC or second USB cable) with a common ground.

To use a different port, edit `dexter_description/urdf/dexter_ros2_control.xacro`:

```xml
<param name="port">/dev/ttyACM0</param>
```

### Gesture Glove (ESP32)

**IMU + flex wiring:**

```
ESP32 DevKit-v1 / DevKitC
├── GPIO 21 (SDA) → MPU6050 SDA
├── GPIO 22 (SCL) → MPU6050 SCL
├── 3V3           → MPU6050 VCC
├── GND           → MPU6050 GND
│
├── 3V3 ──→ flex_sensor ──→ GPIO 34 (analog)
└─────────────────────────→ 47 kΩ ───→ GND
```

- **MPU6050 address:** AD0 → GND (selects `0x68`).
- **BOOT button:** GPIO 0 (internal pull-up; press = LOW). Captures current glove orientation as the new `(0, 0, 0)`.
- **Flex sensor:** ADC-only GPIO 34–39 with a 47 kΩ pulldown to GND.
- **Streaming rate:** 50 Hz (20 ms loop) on port 5005 UDP.

For detailed flash & calibration instructions, see `dexter_firmware/README.md`.

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

*For gesture control tuning, troubleshooting, and the full parameter list, see `dexter_gesture/README.md`.*

*For firmware upload, ESP32 wiring, and calibration procedures, see `dexter_firmware/README.md`.*
