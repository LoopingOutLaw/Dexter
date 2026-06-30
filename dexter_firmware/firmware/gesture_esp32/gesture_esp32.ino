// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 aditya
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
//
// gesture_esp32.ino -- glove-side ESP32 firmware for the Dexter arm.
//
// Reads orientation (roll, pitch, yaw) from an MPU6050 IMU over I²C
// using a complementary filter, reads a flex sensor from an analog
// pin, and streams the four values over WiFi (UDP) to the gesture
// receiver running on the ROS 2 PC. Packet format (newline-terminated):
//
//     roll,pitch,yaw,flex\n          (degrees for first 3, raw ADC for flex)
//
// Press the BOOT button (GPIO 0) once to capture the current glove
// orientation as the new (0,0,0) calibration baseline.
//
// Wiring:
//   MPU6050 VCC → ESP32 3V3
//   MPU6050 GND → ESP32 GND
//   MPU6050 SDA → ESP32 GPIO 21
//   MPU6050 SCL → ESP32 GPIO 22
//   MPU6050 AD0 → ESP32 GND     // sets I²C address to 0x68
//   Flex sensor (voltage divider):
//     3V3 ─→ flex_sensor ─→ ESP32 GPIO 34 ─→ 47 kΩ ─→ GND
//
// Libraries (install via Tools → Manage Libraries):
//   - Adafruit MPU6050
//   - Adafruit Unified Sensor
//
// Board: ESP32 Dev Module (Arduino IDE 2.x).
// Serial Monitor @ 115200 baud.
//
// Streaming rate: 50 Hz (loop period 20 ms) — matches the rate the ROS
// gesture_receiver was tuned for.

#include <Arduino.h>
#include <Wire.h>
#include <WiFi.h>
#include <WiFiUdp.h>

#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>

// ──────────────────────────────────────────────────────────────────
// User configuration (edit these three lines)
// ──────────────────────────────────────────────────────────────────
const char* WIFI_SSID     = "YourNetwork";
const char* WIFI_PASSWORD = "YourPassword";
const char* ROS_PC_IP     = "10.x.x.x";   // run on PC: `hostname -I`

// ──────────────────────────────────────────────────────────────────
// Stream + filter knobs
// ──────────────────────────────────────────────────────────────────
constexpr uint16_t UDP_PORT      = 5005;
constexpr uint16_t LOOP_PERIOD_MS = 20;     // 50 Hz output rate
constexpr float    COMP_ALPHA    = 0.96f;  // gyro weight; lower = trust accel more

// ──────────────────────────────────────────────────────────────────
// IMU + flex pins
// ──────────────────────────────────────────────────────────────────
constexpr uint8_t  FLEX_PIN = 34;          // analog only — GPIO 34..39 are ADC-only

// Flex sensor is currently stubbed at FLEX_OPEN so the receiver has
// something deterministic until the physical sensor is wired.
// Once the sensor is installed (3V3 → flex → FLEX_PIN → 47 kΩ → GND):
//   1. Set HAS_FLEX_SENSOR to 1
//   2. Replace the stub assignment with `int flex_raw = analogRead(FLEX_PIN);`
//   3. Calibrate the open/closed values from the Serial Monitor.
constexpr uint8_t HAS_FLEX_SENSOR = 0;
constexpr int     FLEX_OPEN       = 1800;  // returned while flex is unplugged / stubbed

// ──────────────────────────────────────────────────────────────────
// Calibration zero button — ESP32 BOOT button is GPIO 0 on most
// DevKitC / DevKit-v1 boards. Internal pull-up, press = LOW.
// ──────────────────────────────────────────────────────────────────
constexpr uint8_t CALIB_BUTTON_PIN = 0;
bool _calib_button_was_high = true;        // for edge detection

// ──────────────────────────────────────────────────────────────────
// Globals (kept as a small block; the loop() is short by design)
// ──────────────────────────────────────────────────────────────────
Adafruit_MPU6050 _imu;
WiFiUDP          _udp;

float _gyro_offset_x = 0.0f;
float _gyro_offset_y = 0.0f;
float _gyro_offset_z = 0.0f;

float _roll_deg   = 0.0f;   // around X axis
float _pitch_deg  = 0.0f;   // around Y axis
float _yaw_deg    = 0.0f;   // around Z axis (integrated; will drift)

uint32_t _last_loop_ms = 0;

// ──────────────────────────────────────────────────────────────────
// Helpers
// ──────────────────────────────────────────────────────────────────

// Sample the MPU6050 once. Returns true on success, false if the read
// still failed (we keep the bad sample counts short-circuited above).
bool read_imu(sensors_event_t& accel, sensors_event_t& gyro, sensors_event_t& temp_unused) {
  // Adafruit_MPU6050 spins the I²C bus once and reuses the same buffer;
  // pass `unused` so the SDK doesn't WARN about skipping temp.
  (void)temp_unused;
  return _imu.getEvent(&accel, &gyro, &temp_unused);
}

// Complementary-filter update. We trust the gyro for high-frequency
// motion and the accelerometer-derived tilt (gravity-only) for DC. Yaw
// is gyro-integrated only — bias it once via BOOT to slow the drift.
//
//   angle_new = (1 - alpha) * (angle + gyro_delta)
///            +     alpha * (accel_angle)
static inline float cf_update(
    const float gyro_rate_dps, const float accel_angle_deg,
    const float prev_angle_deg, const float dt_s, const float alpha,
    float& bias_dps) {
  // Standard complementary filter, with a tiny leak applied to bias from
  // the accel-derived residual. The exact leak rate is unimportant
  // because BOOT recalibration gives us a fresh bias at any time.
  const float integrated = prev_angle_deg + gyro_rate_dps * dt_s;
  const float residual   = accel_angle_deg - integrated;
  bias_dps              += residual * 0.001f;        // slow leak
  return integrated + alpha * residual;
}

static inline float rad_to_deg(const float rad) { return rad * 57.2957795f; }
static inline float deg_to_rad(const float deg) { return deg * 0.0174532925f; }

// Build the standard orientation angles from the latest gravity vector
// (Madgwick-lite: roll/pitch only, yaw comes from gyro integration).
void compute_accel_angles(
    const sensors_event_t& accel, float& roll_deg, float& pitch_deg) {
  // Adafruit MPU6050 default orientation: X = forward, Y = left, Z = up.
  // Roll  = atan2(Y, Z)   (around X)
  // Pitch = atan2(-X, hiyp(Y,Z))  (around Y)
  roll_deg  = rad_to_deg(atan2f(accel.acceleration.y,
                                accel.acceleration.z));
  pitch_deg = rad_to_deg(atan2f(-accel.acceleration.x,
                                sqrtf(accel.acceleration.y *
                                      accel.acceleration.y +
                                      accel.acceleration.z *
                                      accel.acceleration.z)));
}

// ──────────────────────────────────────────────────────────────────
// Setup
// ──────────────────────────────────────────────────────────────────
void setup() {
  Serial.begin(115200);
  Serial.setTimeout(1);
  while (!Serial && millis() < 1500) {
    // give Serial Monitor a second to attach so the first lines don't drop
  }

  // ── MPU6050 init ──
  if (!_imu.begin()) {
    Serial.println(F("FATAL: MPU6050 not found on I²C. "
                     "Check wiring (AD0 → GND gives 0x68)."));
    while (true) {
      delay(1000);  // park; do not enter streaming loop
    }
  }
  _imu.setGyroRange(MPU6050_RANGE_500_DEG);
  _imu.setAccelerometerRange(MPU6050_RANGE_4_G);
  _imu.setFilterBandwidth(MPU6050_BAND_21_HZ);

  // ── Calib button ──
  pinMode(CALIB_BUTTON_PIN, INPUT_PULLUP);
  _calib_button_was_high = (digitalRead(CALIB_BUTTON_PIN) == HIGH);

  // ── Take initial gyro bias while the glove is idle. ──
  Serial.println(F("Calibrating gyro bias -- keep glove STILL for ~1 s..."));
  {
    const uint32_t samples = 600;
    float sx = 0.0f, sy = 0.0f, sz = 0.0f;
    sensors_event_t accel, gyro, temp;
    for (uint32_t i = 0; i < samples; ++i) {
      if (read_imu(accel, gyro, temp)) {
        sx += gyro.gyro.x;
        sy += gyro.gyro.y;
        sz += gyro.gyro.z;
      }
      delay(2);
    }
    _gyro_offset_x = rad_to_deg(sx / samples);
    _gyro_offset_y = rad_to_deg(sy / samples);
    _gyro_offset_z = rad_to_deg(sz / samples);
  }

  // ── WiFi join ──
  Serial.print(F("Connecting to WiFi: "));
  Serial.print(WIFI_SSID);
  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);

  uint8_t wifi_attempts = 0;
  while (WiFi.status() != WL_CONNECTED && wifi_attempts < 40) {
    delay(500);
    Serial.print('.');
    ++wifi_attempts;
  }
  if (WiFi.status() != WL_CONNECTED) {
    Serial.println();
    Serial.println(F("FATAL: WiFi did not connect. Check SSID / password."));
    while (true) {
      delay(1000);
    }
  }
  Serial.println();
  Serial.print(F("Connected, IP="));
  Serial.print(WiFi.localIP());
  Serial.print(F("  streaming to "));
  Serial.print(ROS_PC_IP);
  Serial.print(':');
  Serial.println(UDP_PORT);

  // ── Seed current orientation as zero on first boot ──
  {
    sensors_event_t accel, gyro, temp;
    if (read_imu(accel, gyro, temp)) {
      compute_accel_angles(accel, _roll_deg, _pitch_deg);
      Serial.println(F(">>> CALIBRATED -- this position is now (0, 0, 0)"));
    } else {
      Serial.println(F("WARNING: IMU read failed at boot; angles un-initialized."));
    }
  }
  Serial.println(F("Ready!"));
}

// ──────────────────────────────────────────────────────────────────
// Loop -- 50 Hz: read IMU + flex, integrate, send UDP packet.
// ──────────────────────────────────────────────────────────────────
void loop() {
  const uint32_t now = millis();
  if (static_cast<int32_t>(now - _last_loop_ms) < LOOP_PERIOD_MS) {
    return;
  }
  // Clamp large gaps (Serial Monitor pause, brown-out, first tick after
  // setup's gyro-bias + WiFi join) so a single integration step can't
  // lurch the yaw.
  const float dt_s = static_cast<float>(now - _last_loop_ms) * 0.001f;
  _last_loop_ms = now;
  const float dt = (dt_s > 0.1f || dt_s < 0.0f) ? 0.1f : dt_s;

  // ── 1. IMU sample ──
  sensors_event_t accel;
  sensors_event_t gyro;
  sensors_event_t temp_unused;
  if (!read_imu(accel, gyro, temp_unused)) {
    // Skip this tick; we'll retry next loop.
    return;
  }

  // Gyro rates in deg/s (after bias).
  const float gx_dps = rad_to_deg(gyro.gyro.x) - _gyro_offset_x;
  const float gy_dps = rad_to_deg(gyro.gyro.y) - _gyro_offset_y;
  const float gz_dps = rad_to_deg(gyro.gyro.z) - _gyro_offset_z;

  // Accel-derived angles (gravity-only reference).
  float accel_roll_deg  = 0.0f;
  float accel_pitch_deg = 0.0f;
  compute_accel_angles(accel, accel_roll_deg, accel_pitch_deg);

  // Complementary filter for roll/pitch; gyro-integrate yaw.
  static float _roll_bias_dps  = 0.0f;
  static float _pitch_bias_dps = 0.0f;
  _roll_deg   = cf_update(gx_dps, accel_roll_deg,  _roll_deg,  dt, COMP_ALPHA, _roll_bias_dps);
  _pitch_deg  = cf_update(gy_dps, accel_pitch_deg, _pitch_deg, dt, COMP_ALPHA, _pitch_bias_dps);
  _yaw_deg   += gz_dps * dt;

  // ── 2. Flex read ──
  int flex_raw;
#if HAS_FLEX_SENSOR
  flex_raw = analogRead(FLEX_PIN);          // 12-bit, 0..4095 (3V3 reference)
#else
  flex_raw = FLEX_OPEN;                     // stub: glove open at all times.
#endif

  // ── 3. BOOT button = capture current orientation as new (0,0,0). ──
  const bool calib_now_low = (digitalRead(CALIB_BUTTON_PIN) == LOW);
  if (_calib_button_was_high && calib_now_low) {
    // Edge detected: redefine current output
    //   angle = prev_angle - current_filtered_angle    for roll/pitch/yaw.
    _roll_deg   = 0.0f;
    _pitch_deg  = 0.0f;
    _yaw_deg    = 0.0f;
    _roll_bias_dps = 0.0f;
    _pitch_bias_dps = 0.0f;
    Serial.println();
    Serial.println(F(">>> CALIBRATED -- this position is now (0, 0, 0)"));
  }
  _calib_button_was_high = !calib_now_low;

  // ── 4. Pack + send UDP ──
  char buf[96];
  // Format: roll,pitch,yaw,flex\n -- fixed-width to keep the
  // gesture_receiver's split() parser simple.
  const int n = snprintf(buf, sizeof(buf),
                         "%.2f,%.2f,%.2f,%d\n",
                         _roll_deg, _pitch_deg, _yaw_deg, flex_raw);
  if (n > 0 && n < static_cast<int>(sizeof(buf))) {
    _udp.beginPacket(ROS_PC_IP, UDP_PORT);
    _udp.write(reinterpret_cast<const uint8_t*>(buf),
               static_cast<size_t>(n));
    _udp.endPacket();
  }

  // ── 5. Lightweight Serial Monitor telemetry ──
  static uint32_t next_serial_print_ms = 0;
  if (static_cast<int32_t>(now - next_serial_print_ms) >= 0) {
    Serial.print(F("\r  roll="));
    Serial.print(_roll_deg, 1);
    Serial.print(F("°  pitch="));
    Serial.print(_pitch_deg, 1);
    Serial.print(F("°  yaw="));
    Serial.print(_yaw_deg, 1);
    Serial.print(F("°  flex="));
    Serial.print(flex_raw);
    Serial.print(F("   "));
    next_serial_print_ms = now + 100;     // 10 Hz log, no flood
  }
}
