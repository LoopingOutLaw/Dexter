# Dexter Gesture Control — Setup & Usage Guide

Control the Dexter robotic arm by moving your hand.
The ESP32 + MPU6050 on your glove streams orientation data over WiFi (UDP),
and a ROS 2 node maps it directly to arm joint commands.

---

## Architecture

```
[Glove]                      [Your PC]
ESP32 + MPU6050  →  UDP/WiFi  →  gesture_receiver node
                   (port 5005)       ↓
                               arm_controller  (joint_1, 2, 3)
                               gripper_controller (joint_4)
                                     ↓
                               Gazebo simulation / Real robot
```

**No MQTT needed** — raw UDP gives <1 ms latency at 50 Hz, which is
much better than MQTT's ~5–50 ms for real-time arm control.

---

## Hand → Joint Mapping

| Hand Motion            | Axis  | Controls       | Range   |
|------------------------|-------|----------------|---------|
| Wrist twist left/right | Yaw   | Base (joint_1) | ±90°    |
| Wrist tilt fwd/back    | Pitch | Shoulder (j_2) | ±90°    |
| Forearm roll           | Roll  | Elbow (j_3)    | ±90°    |
| Finger bend (flex)     | ADC   | Gripper (j_4)  | open↔closed |

Calibration zero = whatever pose your hand is in when you press BOOT.

---

## Step 1 — ESP32 Firmware

1. Open `gesture_esp32/gesture_esp32.ino` in Arduino IDE.
2. Edit the three lines at the top:
   ```cpp
   const char* WIFI_SSID     = "YourNetwork";
   const char* WIFI_PASSWORD = "YourPassword";
   const char* ROS_PC_IP     = "10.x.x.x";   // run: hostname -I
   ```
3. Install libraries (Tools → Manage Libraries):
   - **Adafruit MPU6050**
   - **Adafruit Unified Sensor**
4. Select board: **ESP32 Dev Module**
5. Flash. Open Serial Monitor at 115200 baud.
6. You should see gyro bias calibration, then `Ready!`

### Wiring (unchanged from your original)
```
MPU6050 VCC  → ESP32 3.3V
MPU6050 GND  → ESP32 GND
MPU6050 SDA  → ESP32 GPIO 21
MPU6050 SCL  → ESP32 GPIO 22
MPU6050 AD0  → ESP32 GND
```

---

## Step 2 — ROS 2 Package

### Install into your workspace

```bash
# Copy dexter_gesture/ into your ros2 workspace
cp -r dexter_gesture/  ~/ros2_ws/src/

# Build
cd ~/ros2_ws
colcon build --packages-select dexter_gesture
source install/setup.bash
```

---

## Step 3 — Run

### Option A: Full simulation in one command (recommended)
```bash
ros2 launch dexter_gesture gesture_sim.launch.py
```
This starts Gazebo + controllers + MoveIt + gesture receiver.

### Option B: Separate terminals (useful for debugging)

**Terminal 1** — Simulation:
```bash
ros2 launch dexter_bringup simulated_robot.launch.py
```

**Terminal 2** — Gesture receiver:
```bash
ros2 launch dexter_gesture gesture_control.launch.py
```

### Option C: Test WITHOUT the physical ESP32

Run the UDP simulator on your PC:
```bash
python3 ~/ros2_ws/src/dexter_gesture/udp_test_sender.py
```
This sends sine-wave gesture data so you can verify the arm moves
before the glove is ready. Use `--manual` to type specific angles.

---

## Step 4 — Calibration

1. Put the glove on. Hold your hand in your **neutral/relaxed** position
   (the pose where you want the arm to be at rest).
2. Press the **BOOT button** on the ESP32.
3. Serial monitor prints: `>>> CALIBRATED — this position is now (0, 0, 0)`
4. Now move your hand — the arm follows.

---

## Tuning Parameters

Edit `config/gesture_config.yaml` or pass as launch arguments:

```bash
# More smoothing (less shaky, more lag)
ros2 launch dexter_gesture gesture_control.launch.py smoothing_alpha:=0.15

# Less smoothing (more responsive, may shake)
ros2 launch dexter_gesture gesture_control.launch.py smoothing_alpha:=0.4

# If arm moves opposite to your hand, flip an axis
ros2 launch dexter_gesture gesture_control.launch.py invert_pitch:=true
```

| Parameter             | Default | Effect                                      |
|-----------------------|---------|---------------------------------------------|
| `smoothing_alpha`     | 0.25    | Filter strength (lower = smoother but slower)|
| `deadband_deg`        | 0.5     | Ignore hand tremor smaller than this (°)    |
| `trajectory_duration_ms` | 80  | Speed of each motion command (ms)           |
| `invert_roll/pitch/yaw` | false | Flip direction of each axis                |

---

## Flex Sensor (when it arrives)

1. Wire it: `3.3V → flex_sensor → GPIO 34 → 47kΩ → GND`
2. In `gesture_esp32.ino`, uncomment:
   ```cpp
   int flex_raw = analogRead(FLEX_PIN);  // line ~155
   ```
   And comment out or delete:
   ```cpp
   int flex_raw = FLEX_OPEN;             // stub line
   ```
3. Calibrate open/closed ADC values:
   - Open hand → note the Serial Monitor `Flex:` value → set `flex_open_adc`
   - Closed fist → note value → set `flex_closed_adc`
4. Update in `gesture_config.yaml` or as launch args:
   ```bash
   ros2 launch dexter_gesture gesture_control.launch.py \
       flex_open_adc:=1750.0 flex_closed_adc:=3400.0
   ```

---

## Troubleshooting

### Arm doesn't move
- Check `ros2 topic echo /arm_controller/joint_trajectory` — are messages arriving?
- Check `ros2 node info /gesture_receiver` for subscribers.
- Run `udp_test_sender.py` to rule out ESP32 issues.

### "No UDP packets received" warning
- Confirm ESP32 Serial Monitor shows it's connected to WiFi.
- Check your PC IP matches `ROS_PC_IP` in the firmware.
- Make sure firewall isn't blocking port 5005:
  ```bash
  sudo ufw allow 5005/udp   # Ubuntu
  ```

### Arm shakes / oscillates
- Increase `deadband_deg` (try 1.5–2.0).
- Decrease `smoothing_alpha` (try 0.1–0.15).
- Increase `trajectory_duration_ms` (try 120–150).

### Yaw drifts over time
- Press BOOT to recalibrate.
- Reduce `COMP_ALPHA` in firmware (try 0.94) to trust accel more.

### Wrong direction
- Use `invert_roll`, `invert_pitch`, or `invert_yaw` launch arguments.
