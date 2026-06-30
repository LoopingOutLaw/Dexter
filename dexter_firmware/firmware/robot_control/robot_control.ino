#include <Servo.h>

// Declare the Arduino pin where each servo is connected
#define SERVO_BASE_PIN 8
#define SERVO_SHOULDER_PIN 9
#define SERVO_ELBOW_PIN 10
#define SERVO_GRIPPER_PIN 11

// Define the start configuration of the joints
#define BASE_START 90
#define SHOULDER_START 90
#define ELBOW_START 90
#define GRIPPER_START 0

// Register the servo motors of each joint
Servo base;  
Servo shoulder;  
Servo elbow;  
Servo gripper; 

uint8_t idx = 0;
uint8_t val_idx = 0;
char value[4] = "000";


/*
 * Apply a safe command to a servo.
 *   - Clamps the goal into the absolute servo range (0..180)
 *   - Skips writes if the new goal matches what the servo already holds
 *     (eliminates I2C chatter and pulse stretching on busy bus)
 *
 * Smoothing is handled upstream in ROS 2 (gesture_receiver ->
 * S-curve smoother -> JointTrajectory -> joint_trajectory_controller),
 * so this firmware does NOT ramp one degree at a time. Looping with
 * delay() here would re-introduce the very jerk we are killing in
 * software and desynchronize from the controller's 50 Hz command rate.
 */
int clamp_servo_angle(int goal) {
  if (goal < 0)   return 0;
  if (goal > 180) return 180;
  return goal;
}

void apply_goal(Servo& motor, int goal) {
  goal = clamp_servo_angle(goal);
  if (goal != motor.read()) {
    motor.write(goal);
  }
}

void reach_goal(Servo& motor, int goal) {
  apply_goal(motor, goal);
}

void setup() {
  // Attach and Initialize each Servo to the Arduino pin where it is connected
  base.attach(SERVO_BASE_PIN);
  shoulder.attach(SERVO_SHOULDER_PIN);
  elbow.attach(SERVO_ELBOW_PIN);
  gripper.attach(SERVO_GRIPPER_PIN); 

  // Set a common start point for each joint
  // This way, the start status of each joint is known
  base.write(BASE_START);
  shoulder.write(SHOULDER_START);
  elbow.write(ELBOW_START);
  gripper.write(GRIPPER_START);

  // Start the Serial communication with ROS
  Serial.begin(115200);
  Serial.setTimeout(1);
}

void loop() {
  if (Serial.available())
  {
    char chr = Serial.read();

    // base motor
    if(chr == 'b')
    {
      idx = 0;
      val_idx = 0;
    }
    // shoulder motor
    else if(chr == 's')
    {
      idx = 1;
      val_idx = 0;
    }
    // elbow motor
    else if(chr == 'e')
    {
      idx = 2;
      val_idx = 0;
    }
    // gripper motor
    else if(chr == 'g')
    {
      idx = 3;
      val_idx = 0;
    }
    // Separator
    else if(chr == ',')
    {
      int val = atoi(value);
      if(idx == 0)
      {
        reach_goal(base, val);
      }
      else if(idx == 1)
      {
        reach_goal(shoulder, val);
      }
      else if(idx == 2)
      {
        reach_goal(elbow, val);
      }
      else if(idx == 3)
      {
        reach_goal(gripper, val);
      }

      // reset the angle
      value[0] = '0';
      value[1] = '0';
      value[2] = '0';
      value[3] = '\0';
    }
    // Plain number
    else
    {
      value[val_idx] = chr;
      val_idx++;
    }
  }
}
