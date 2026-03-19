// ============================================================================
// ISOLATED FUNCTION TEST SKETCH
// ============================================================================
// This sketch allows you to test individual functions from controls.ino
// in isolation using Serial commands.
//
// Usage: Open Serial Monitor (9600 baud) and type commands like:
//   HELP              - Show all available test commands
//   MOTORS <L> <R>    - Test moveMotors with left and right speeds (0-255)
//   STOP              - Test stopMotors
//   STRAIGHT <S> <B>  - Test moveMotorsStraight (speed, backward 0/1)
//   GRIPPER <G>       - Test executeGripper (grab 0/1)
//   PD <O> <D>        - Test runPDLogic with offset and direction
//   ARC <S> <R> <L>   - Test executeArc (outerSpeed, ratio, isLeft 0/1)
//   GRAB <X> <Y>      - Test executeGrabSequence (xOffset, yOffset)
//   DROP              - Test executeDropSequence
//   MAG               - Test calibrateMagnetometer
//   HEADING <G>       - Test getFusedHeading with gyro_z value
//   HEADING_CHANGE    - Test getHeadingChange with target angle
// ============================================================================

#include <Wire.h>
#include <Adafruit_LSM6DSOX.h>
#include <Adafruit_LIS3MDL.h>
#include <Adafruit_Sensor.h>
#include <Servo.h>

// ============================================================================
// HARDWARE CONFIGURATION (Same as controls.ino)
// ============================================================================

// Motor A (Right)
int pwma = 6;
int ain1 = 8;
int ain2 = 7;

// Motor B (Left)
int pwmb = 5;
int bin1 = 2;
int bin2 = 3;

// Servos
Servo gripperServo;  // Pin 10
Servo liftServo;     // Pin 11

// IMU
Adafruit_LSM6DSOX lsm6ds;
Adafruit_LIS3MDL lis3mdl;

// ============================================================================
// CONTROL PARAMETERS (Same as controls.ino)
// ============================================================================

int baseSpeed = 150;
float Kp = 0.5;
float Kd = 0;
int lastError = 0;

int consecutiveTurnCount = 0;
const int turnThreshold = 10;

volatile int grabXOffset = 0;
volatile int grabYOffset = 0;
int grabPhase = 0;
unsigned long grabPhaseTime = 0;

const int GRAB_BASE_SPEED = 80;
const int GRAB_FINAL_PUSH_TIME = 2000;
const int GRAB_X_THRESHOLD = 3;
const int grabThreshold = 5;
float grabKp = 0.5;
float grabKd = 0;

int dropPhase = 0;
unsigned long dropPhaseTime = 0;

const int DROP_MOTOR_DELAY_TIME = 1000;
const int dropThreshold = 5;

float mag_offset_x = 0, mag_offset_y = 0, mag_offset_z = 0;
float mag_scale_x = 1, mag_scale_y = 1, mag_scale_z = 1;
float current_heading = 0;
const float heading_alpha = 0.60;

// ============================================================================
// TEST VARIABLES - EDIT THESE TO CHANGE TEST BEHAVIOR
// ============================================================================

int testSpeed = 150;             // Speed to move forward (0-255)
unsigned long testDuration = 5000; // Duration in milliseconds (ms)
unsigned long testStartTime = 0;
bool testStarted = false;

// ============================================================================
// SETUP
// ============================================================================

void setup() {
  Serial.begin(9600);
  Serial.println("\n\n=== FORWARD MOVEMENT TEST ===");
  Serial.print("Speed: ");
  Serial.print(testSpeed);
  Serial.print(", Duration: ");
  Serial.print(testDuration);
  Serial.println(" ms");
  
  // Initialize motor pins
  pinMode(pwma, OUTPUT);
  pinMode(pwmb, OUTPUT);
  pinMode(ain1, OUTPUT);
  pinMode(ain2, OUTPUT);
  pinMode(bin1, OUTPUT);
  pinMode(bin2, OUTPUT);
  
  // Initialize servos
  gripperServo.attach(10);
  liftServo.attach(11);
  gripperServo.write(90);
  liftServo.write(90);
  
  // Initialize IMU
  if (!lsm6ds.begin_I2C()) {
    Serial.println("LSM6DSOX not found!");
  } else {
    Serial.println("LSM6DSOX initialized");
  }
  
  if (!lis3mdl.begin_I2C()) {
    Serial.println("LIS3MDL not found!");
  } else {
    Serial.println("LIS3MDL initialized");
  }
}

// ============================================================================
// MAIN LOOP
// ============================================================================

void loop() {
  // Start test on first iteration
  if (!testStarted) {
    Serial.println("\n[START] Moving forward...");
    testStartTime = millis();
    testStarted = true;
    moveMotorsStraight(testSpeed, false);
  }
  
  // Check if test duration has elapsed
  unsigned long elapsedTime = millis() - testStartTime;
  if (elapsedTime >= testDuration) {
    stopMotors();
    Serial.print("[COMPLETE] Movement finished after ");
    Serial.print(elapsedTime);
    Serial.println(" ms");
    Serial.print("Distance estimate: ~");
    Serial.print((elapsedTime / 1000.0) * 5.0 * (testSpeed / 150.0));
    Serial.println(" cm");
    
    // Loop forever after test completes
    while(1) {
      delay(1000);
    }
  }
}

// ...existing code...

void moveMotors(int left, int right) {
  // Clamp values
  left = constrain(left, 0, 255);
  right = constrain(right, 0, 255);
  
  // Left motor (Motor B)
  digitalWrite(bin1, LOW);
  digitalWrite(bin2, HIGH);
  analogWrite(pwmb, left);
  
  // Right motor (Motor A)
  digitalWrite(ain1, LOW);
  digitalWrite(ain2, HIGH);
  analogWrite(pwma, right);
}

void stopMotors() {
  digitalWrite(ain1, LOW);
  digitalWrite(ain2, LOW);
  digitalWrite(bin1, LOW);
  digitalWrite(bin2, LOW);
  analogWrite(pwma, 0);
  analogWrite(pwmb, 0);
}

void moveMotorsStraight(int speed, bool backward) {
  speed = constrain(speed, 0, 255);
  
  if (backward) {
    digitalWrite(ain1, HIGH);
    digitalWrite(ain2, LOW);
    digitalWrite(bin1, HIGH);
    digitalWrite(bin2, LOW);
  } else {
    digitalWrite(ain1, LOW);
    digitalWrite(ain2, HIGH);
    digitalWrite(bin1, LOW);
    digitalWrite(bin2, HIGH);
  }
  
  analogWrite(pwma, speed);
  analogWrite(pwmb, speed);
}

void executeGripper(bool grab) {
  if (grab) {
    gripperServo.write(0);   // Close gripper
  } else {
    gripperServo.write(180); // Open gripper
  }
}

void runPDLogic(int offset, int dir) {
  unsigned long startTime = millis();
  int error = offset;
  int correction = (int)(error * Kp + (error - lastError) * Kd);
  
  int leftSpeed = baseSpeed;
  int rightSpeed = baseSpeed;
  
  if (dir == 1) {
    rightSpeed -= correction;
  } else {
    leftSpeed -= correction;
  }
  
  leftSpeed = constrain(leftSpeed, 0, 255);
  rightSpeed = constrain(rightSpeed, 0, 255);
  
  moveMotors(leftSpeed, rightSpeed);
  lastError = error;
}

void executeArc(int outerSpeed, float ratio, bool isLeft) {
  outerSpeed = constrain(outerSpeed, 0, 255);
  ratio = constrain(ratio, 0.0f, 1.0f);
  
  int innerSpeed = (int)(outerSpeed * ratio);
  
  if (isLeft) {
    moveMotors(outerSpeed, innerSpeed);
  } else {
    moveMotors(innerSpeed, outerSpeed);
  }
}

void executeGrabSequence(int xOffset, int yOffset) {
  grabPhase = 0;
  grabPhaseTime = millis();
}

void executeDropSequence() {
  dropPhase = 0;
  dropPhaseTime = millis();
}

void calibrateMagnetometer() {
  // Placeholder - actual calibration code from controls.ino
  Serial.println("Magnetometer calibration started...");
  delay(1000);
  mag_offset_x = 0;
  mag_offset_y = 0;
  mag_offset_z = 0;
  Serial.println("Calibration complete!");
}

float getFusedHeading(float gyro_z) {
  float mag_heading = 0;
  
  // Read magnetometer
  sensors_event_t mag;
  lis3mdl.getEvent(&mag);
  
  float mx = mag.magnetic.x - mag_offset_x;
  float my = mag.magnetic.y - mag_offset_y;
  
  mx *= mag_scale_x;
  my *= mag_scale_y;
  
  mag_heading = atan2(my, mx) * (180.0 / PI);
  if (mag_heading < 0) mag_heading += 360;
  
  // Complementary filter
  current_heading = heading_alpha * (current_heading + gyro_z) + (1.0 - heading_alpha) * mag_heading;
  
  if (current_heading < 0) current_heading += 360;
  if (current_heading >= 360) current_heading -= 360;
  
  return current_heading;
}

float getHeadingChange(float target_angle) {
  float diff = target_angle - current_heading;
  
  // Normalize to -180 to 180
  if (diff > 180) diff -= 360;
  if (diff < -180) diff += 360;
  
  return diff;
}
