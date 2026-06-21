# dexter_firmware

Arduino & ESP32 firmware for the Dexter robotic arm.

---

## 📂 Sketches

| Sketch | Board | Purpose |
|---|---|---|
| `robot_control/robot_control.ino` | Arduino Uno | Main servo driver; accepts serial commands from `dexter_controller` |
| `gesture_esp32/gesture_esp32.ino` | ESP32 DevKit | Glove-side IMU + flex sensor; streams UDP orientation packets |
| `motor_calibration/motor_calibration.ino` | Arduino Uno | Helper for finding servo PWM limits |
| `simple_serial_receiver.ino` | — | Minimal serial listener for debugging |
| `simple_serial_transmitter.ino` | — | Minimal serial sender for debugging |
| `simple_servo_control.ino` | — | Basic servo sweep for testing |

---

## 🔧 robot_control — Arduino Uno (Real Robot)

**Wiring:**

```
Arduino Uno
├── D8  → Base servo     (signal; VCC/GND from separate 5V supply)
├── D9  → Shoulder servo
├── D10 → Elbow servo
└── D11 → Gripper servo

USB Serial: /dev/ttyACM0 @ 115200 baud
```

> ⚠️ **Power:** Servos draw 0.5–1 A under load. Do not power them from the Arduino 5V pin. Use an external 5V supply with a common ground.

**Upload:**

1. Open `robot_control/robot_control.ino` in Arduino IDE 2.x.
2. Select **Tools → Board → Arduino Uno**.
3. Select the correct serial port.
4. Click **Upload**.
5. Open the Serial Monitor @ 115200 baud to verify the handshake message.

---

## 📡 gesture_esp32 — ESP32 DevKit (Gesture Glove)

### Wiring

```
ESP32 DevKit-v1 / DevKitC
├── GPIO 21 (SDA) ──► MPU6050 SDA
├── GPIO 22 (SCL) ──► MPU6050 SCL
├── 3V3           ──► MPU6050 VCC
├── GND           ──► MPU6050 GND  (AD0 → GND selects address 0x68)
│
├── 3V3 ──► Flex sensor ──► GPIO 34 (analog)
└──────────► 47 kΩ ────────► GND
```

| Component | Pin | Notes |
|---|---|---|
| MPU6050 SDA | GPIO 21 | Hardware I²C |
| MPU6050 SCL | GPIO 22 | Hardware I²C |
| MPU6050 VCC | 3V3 | |
| MPU6050 GND | GND | Tie AD0 to GND for address 0x68 |
| Flex sensor | GPIO 34 | ADC-only pins 34–39 used |
| BOOT button | GPIO 0 | Internal pull-up; press = LOW |

### Required Libraries (Arduino IDE)

Install via **Tools → Manage Libraries**:

- `Adafruit MPU6050`
- `Adafruit Unified Sensor`

Also install ESP32 board support: **Tools → Board → Boards Manager** → search `ESP32` by Espressif Systems.

### Configuration

Edit the three lines near the top of `gesture_esp32.ino`:

```cpp
const char* WIFI_SSID     = "YourNetwork";   // <-- your WiFi name
const char* WIFI_PASSWORD = "YourPassword";  // <-- your WiFi password
const char* ROS_PC_IP     = "192.168.1.42";  // <-- `hostname -I` on your PC
```

### Upload

1. Open `gesture_esp32/gesture_esp32.ino` in Arduino IDE 2.x.
2. Select **Tools → Board → ESP32 Dev Module**.
3. Select the correct serial port (e.g., `/dev/ttyUSB0` or `/dev/ttyACM0`).
4. Click **Upload**.
5. Wait until you see `Connected, IP=…  streaming to …:5005` in the Serial Monitor @ **115200 baud**.

### Calibration

- **BOOT button (GPIO 0):** Press once. The current glove orientation becomes the new `(0, 0, 0)`.
- **Serial check:** After flashing, the Serial Monitor prints the current orientation every 100 ms. Hold the glove in the desired zero position and press BOOT.

### Enabling the Flex Sensor

The flex sensor is **stubbed off** by default (`HAS_FLEX_SENSOR = 0`) so the firmware is deterministic while you wire the IMU. Once the flex sensor is installed:

1. Set `constexpr uint8_t HAS_FLEX_SENSOR = 1;` in the sketch.
2. Re-upload.
3. Open/close your hand and note the ADC values in the Serial Monitor.
4. Update `flex_open_adc` / `flex_closed_adc` in your ROS launch file accordingly.

---

## 🛠️ motor_calibration — Arduino Uno

Use this helper to find the exact PWM range for each servo before running the main controller.

1. Upload `motor_calibration/motor_calibration.ino`.
2. The sketch steps each servo through 0°, 90°, 180°.
3. Note any mechanical limits and update `robot_control.ino` if needed.

---

## 📋 Serial Protocol (robot_control ↔ ROS 2)

The Arduino accepts commands of the form:

```
b<angle>,s<angle>,e<angle>,g<angle>,
```

Each angle is zero-padded to 3 digits: `b090,s090,e090,g000,`

- `b` = base (joint 1)
- `s` = shoulder (joint 2)
- `e` = elbow (joint 3)
- `g` = gripper (joint 4)

Sent at **50 Hz** by `dexter_controller` when the robot is live.

---

## 🔍 Troubleshooting

### Arduino: `avrdude: stk500_recv()` on upload

Double-check the serial port. Some clones use a CH340/CP2102 and show up as `/dev/ttyUSB0` instead of `/dev/ttyACM0`.

### ESP32: `FATAL: MPU6050 not found on I²C`

- Verify the 4 I²C wires (SDA, SCL, VCC, GND).
- Ensure AD0 is tied to GND (address 0x68).
- Try swapping SDA/SCL — some breadboards have reversed labels.

### ESP32: `FATAL: WiFi did not connect`

- Double-check SSID and password spelling.
- Ensure the PC and ESP32 are on the same subnet.
- Look at router logs for the ESP32 MAC address.

### ESP32: Packets not arriving at PC

```bash
# On the PC (Ubuntu/Debian)
sudo ufw allow 5005/udp
# or
cat /proc/sys/net/ipv4/ip_forward   # should be 1 for multicast, usually not needed for UDP unicast
```

### Servo jitter / brown-out

The Arduino's 5V regulator cannot source the ~1 A peak current of 4 servos. Use an external 5V supply (UBEC or second USB cable) and tie all grounds together.

---

*For ROS 2 launch files and gesture tuning, see `dexter_gesture/README.md`.*
