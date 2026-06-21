# dexter_gesture

ROS 2 node that turns glove-orientation packets from the ESP32 into
smooth, jerk-bounded joint-trajectory commands for the Dexter arm and
gripper.

Hand-wrist motion is captured by an MPU6050 strapped to a glove,
streamed at ~50 Hz over WiFi (raw UDP), and turned into pose commands
for `arm_controller` and `gripper_controller` at a separate 100 Hz tick.

## Architecture

```
[ESP32 + MPU6050]  ---UDP/WiFi--->  gesture_receiver node
   roll,pitch,yaw,flex                 ┊  (4-stage smoothing, see below)
   (newline-terminated)                ↓
                                ┌──────┴──────┐
                                ↓             ↓
                  /arm_controller/      /gripper_controller/
                  joint_trajectory      joint_trajectory
                                ↓             ↓
                           Gazebo / real robot
```

UDP is used instead of MQTT because round-trips are typically <1 ms at
50 Hz on a home WiFi router — much better than MQTT's 5–50 ms for
real-time arm control.

## Hand → Joint mapping

The glove's three orientation axes are fused into a 4-DOF joint target
inside the receiver. The two tables below keep calibration non-ambiguous.

| Hand motion            | Source axis | Drives                        | Range       |
|------------------------|-------------|-------------------------------|-------------|
| Wrist twist (yaw)      | Yaw         | Base (joint_1) directly       | ± 90°       |
| Wrist tilt (pitch)     | Pitch       | Reach — shoulder + elbow     | ± 90°       |
| Forearm roll           | Roll        | Elbow fine-tune (j_3 offset)  | ± 90°       |
| Finger bend (flex)     | ADC value   | Gripper (joint_4)             | open↔closed |

Pitch is split into shoulder and elbow gains (default `1.0` and `0.6`)
so a single tilt produces a coordinated lift, not two independent joint
moves.  Roll is added as a small offset (× 0.4) onto joint_3 rather than
being its own DOF — that matches the gripper orientation you want when
the wrist rolls.

Calibration zero is whatever pose you hold when the ESP32 BOOT button
is pressed; see `dexter_firmware/README.md` for the exact procedure.

## Smoothing pipeline (4 stages)

```
raw UDP packet
   │   "roll,pitch,yaw,flex\n"
   ↓
┌──────────────────┐
│ Stage A  Deadband│  ignore |Δθ| < 0.5°     — kills sensor jitter
└──────────────────┘
   ↓
┌──────────────────┐
│ Stage B  Low-pass│  EMA with α = 0.12      — softs the gesture
└──────────────────┘
   ↓
┌──────────────────┐
│ Stage C  S-curve │  jerk-bounded,
│         @100 Hz  │  v ≤ 1.2 rad/s,
│  (wall-clock     │  a ≤ 6.0 rad/s²,
│   timer, not UDP)│  j ≤ 8.0 rad/s³
└──────────────────┘
   ↓
┌──────────────────┐
│ Stage D  Publish │  publish only if |Δq| > 0.005 rad
│         deadband  │  (saves bus bandwidth at idle)
└──────────────────┘
   ↓
/arm_controller/joint_trajectory     (joint_1, joint_2, joint_3)
/gripper_controller/joint_trajectory (joint_4)
```

Stages A/B run on the UDP receive thread (one event per packet).
Stages C/D run on a 10 ms wall-clock timer — decoupled from UDP rate,
so a 200 ms packet hiccup never makes the arm hop.

The planner is a third-order jerk-bounded smoother implemented in
`motion_smoother.py`.  Three independent instances run for the arm
joints, and a fourth with softer (lower v/a/j) caps runs for the
gripper so you don't flick the fingers shut.

## Files in this package

```
dexter_gesture/
├── dexter_gesture/
│   ├── __init__.py
│   ├── gesture_receiver.py   # the ROS 2 node + 4-stage pipeline
│   ├── motion_smoother.py    # JointSmoother profile (jerk-bounded)
│   └── udp_test_sender.py    # PC-side simulator (no ESP32 needed)
├── launch/
│   ├── gesture_control.launch.py  # node only (ros2_control must be up)
│   └── gesture_sim.launch.py      # full stack: Gazebo + ctrls + node
├── config/
│   └── gesture_config.yaml        # all defaults live here
├── test/
│   ├── test_motion_smoother.py    # JointSmoother unit tests
│   └── test_gesture_receiver.py   # node tests via FakeReceiver stub
└── README.md
```

## Running

### Quick start (everything in one terminal)

```bash
# from your colcon workspace root
colcon build --packages-select dexter_gesture
source install/setup.bash
ros2 launch dexter_gesture gesture_sim.launch.py
```

This brings up Gazebo, the controllers, MoveIt, and the gesture
receiver.  The receiver is delayed 5 s so controllers are ready first.

### Without the ESP32 (use the simulator)

```bash
# terminal 1
ros2 launch dexter_gesture gesture_control.launch.py

# terminal 2  -- PC-side UDP simulator
python3 install/dexter_gesture/lib/python3.12/site-packages/dexter_gesture/udp_test_sender.py
# (or pass --ip <receiver-ip> --port 5005)
```

Three simulated modes are available:

```bash
udp_test_sender.py --sine    # default — smooth sweep
udp_test_sender.py --static  # zeros — arm should hold home
udp_test_sender.py --manual  # type roll pitch yaw [flex] interactively
```

### Without Gazebo (real arm)

Just launch the gesture node and let it talk to your real controllers:

```bash
ros2 launch dexter_gesture gesture_control.launch.py \
    ~udp_port:=5005
```

## Tuning parameters

All defaults are in `config/gesture_config.yaml`; every one is also
exposed as a launch argument so you can try values without editing
files.

### Arm motion (joints 1–3)

Launch argument        | Default | Effect
-----------------------|---------|----------------------------------------------
`low_pass_alpha`       | 0.12    | EMA blend: 0 = hold last, 1 = no filter
`deadband_deg`         | 0.5     | suppress sensor jitter smaller than this
`max_velocity_rad_s`   | 1.2     | speed cap on each joint
`max_acceleration_rad_s2` | 6.0  | acceleration cap
`max_jerk_rad_s3`      | 8.0     | jerk cap (this is what the smoother enforces)
`settling_time_s`      | 0.15    | how quickly the smoother eases below caps
`smoother_rate_hz`     | 100     | wall-clock step of the S-curve profiler
`publish_deadband_rad` | 0.005   | suppress topic publishes below this delta

### Gripper motion (joint_4)

Roughly half the v/a/j of the arm so you don't flick the gripper open
or shut at full speed.  The defaults are tuned for an MG90S-style
servo on a 90° pivot.

Launch argument                  | Default | Effect
---------------------------------|---------|------------------
`gripper_max_velocity_rad_s`     | 0.8     | gripper speed cap
`gripper_max_acceleration_rad_s2`| 4.0     | gripper accel cap
`gripper_max_jerk_rad_s3`        | 5.0     | gripper jerk cap
`gripper_settling_time_s`        | 0.20    | gripper settling time

### Trajectory shape and sign

Launch argument             | Default | Effect
----------------------------|---------|--------------------------------
`trajectory_duration_ms`    | 80      | per-point time-from-start
`invert_roll / pitch / yaw` | false   | flip one axis if your glove is mounted backwards

### Flex-to-gripper calibration

Launch argument     | Default | Effect
--------------------|---------|----------------------------------------------
`flex_open_adc`     | 2900.0  | ADC reading when fingers are open
`flex_closed_adc`   | 3700.0  | ADC reading at closed fist
`shoulder_gain`     | 1.0     | pitch→shoulder scaling
`elbow_gain`        | 0.6     | pitch→elbow scaling

### Common recipes

```bash
# Shakier glove / older user — extra smoothing
ros2 launch dexter_gesture gesture_control.launch.py \
    low_pass_alpha:=0.08 deadband_deg:=1.2 \
    max_jerk_rad_s3:=5.0 max_acceleration_rad_s2:=4.0

# Snappy arm, no smoothing
ros2 launch dexter_gesture gesture_control.launch.py \
    low_pass_alpha:=0.30 deadband_deg:=0.2 \
    max_jerk_rad_s3:=15.0 max_acceleration_rad_s2:=10.0

# Arm moves opposite of hand
ros2 launch dexter_gesture gesture_control.launch.py invert_pitch:=true

# Override UDP port
ros2 launch dexter_gesture gesture_control.launch.py udp_port:=5006
```

## Tests

```bash
colcon build --packages-select dexter_gesture --symlink-install
colcon test --packages-select dexter_gesture
colcon test-result --all
```

`colcon test` runs the pytest suites under `test/`:

- `test_motion_smoother.py` — profile shape, cap enforcement,
  settling, and convergence from the worst-case initial state.
- `test_gesture_receiver.py` — UDP parsing, deadband, EMA, integration
  through the smoother, publish deadband, and the watchdog liveness
  counter.  Uses a sys-modules stub for `rclpy` so the test never
  spins up a real node.

## Troubleshooting

**Arm doesn't move.** Open `ros2 topic echo /arm_controller/joint_trajectory`
and confirm messages arrive.  With nothing connected, run
`udp_test_sender.py --sine` to verify the receiver end-to-end.

**"No UDP packets received" warning.** Check the ESP32 Serial Monitor
for WiFi join.  Confirm the PC IP in the firmware matches
`hostname -I`.  Allow the port on your firewall:

```bash
sudo ufw allow 5005/udp   # Ubuntu/Debian
# or temporarily:
sudo ufw disable
```

**Arm shakes.** Raise `deadband_deg` to 1.5–2.0, lower
`low_pass_alpha` to 0.08, and/or lower `max_jerk_rad_s3` to 5.0.

**Jerk on first motion.** You may have a large initial target jump
(cold bias).  Lower `settling_time_s` to 0.20 or simply re-home the
arm with BOOT to recalibrate.

**Yaw drifts over time.** Re-home by pressing BOOT.  In the firmware
reduce `COMP_ALPHA` (default 0.98) toward 0.94 to lean more on the
accel and less on gyro integration.

**Wrong direction.** Use `invert_roll`, `invert_pitch`, or `invert_yaw`.

## Related docs

- Spec: `~/Dexter/docs/superpowers/specs/2026-06-15-dexter-motion-smoothing-design.md`
- Plan: `~/Dexter/docs/superpowers/plans/2026-06-15-dexter-motion-smoothing.md`
- Firmware & wiring: `~/Dexter/dexter_firmware/README.md`
- Cross-package launch: `~/Dexter/dexter_gesture/launch/gesture_sim.launch.py`
