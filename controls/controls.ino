#include <Wire.h>
#include <Adafruit_LSM6DSOX.h>
#include <Adafruit_LIS3MDL.h>
#include <Adafruit_Sensor.h>
#include <Servo.h>

// ============================================================================
// HARDWARE CONFIGURATION
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
// I2C & COMMUNICATION
// ============================================================================

const int ARDUINO_I2C_ADDR = 0x08;
volatile int currentCommand = 0;
int previousCommand = 0;  // Track command before transition
volatile int i2cOffset = 0;
volatile int i2cDirection = 0;
unsigned long lastHeartbeat = 0;
unsigned long lastTurnBackupTime = 0;  // Track when turn+backup completes

// ============================================================================
// LINE FOLLOWING (PD CONTROL)
// ============================================================================

int baseSpeed = 150;
float Kp = 0.5;
float Kd = 0;
int lastError = 0;

// ============================================================================
// TURN TRACKING
// ============================================================================

int consecutiveTurnCount = 0;
const int turnThreshold = 10;
const int TURN_BACKUP_TIME = 7000;    // 7 second backup duration 
const int TURN_BACKUP_SPEED = 100;    // Backup speed
const int TURN_SETTLE_FROM_FOLLOW = 5750;  // 5.75s settling after line-follow
const int TURN_SETTLE_FROM_BACKUP = 500;   // 0.5s settling after backup
const int FOLLOW_IGNORE_TIME = TURN_SETTLE_FROM_BACKUP;  // Ignore follow commands during backup settling (prevent errant corrections)
// ============================================================================
// GRAB SEQUENCE
// ============================================================================

volatile int grabXOffset = 0;
volatile int grabYOffset = 0;
int grabPhase = 0;
unsigned long grabPhaseTime = 0;
int grabCommandCount = 0;
int lastGrabError = 0;

const int GRAB_BASE_SPEED = 80;
const int GRAB_FINAL_PUSH_TIME = 2000;
const int GRAB_X_THRESHOLD = 3;
const int grabThreshold = 5;
float grabKp = 0.5;
float grabKd = 0;

bool grabSequenceCompleted = false;

// ============================================================================
// DROP SEQUENCE
// ============================================================================

int dropPhase = 0;
unsigned long dropPhaseTime = 0;
int dropCommandCount = 0;

const int DROP_MOTOR_DELAY_TIME = 1000;
const int dropThreshold = 5;

bool dropSequenceCompleted = false;

// ============================================================================
// MAGNETOMETER CALIBRATION & FUSION
// ============================================================================

// Magnetometer calibration offsets and scales
float mag_offset_x = 0, mag_offset_y = 0, mag_offset_z = 0;
float mag_scale_x = 1, mag_scale_y = 1, mag_scale_z = 1;

// Heading fusion
float current_heading = 0;      // Fused heading in degrees (0-360)
unsigned long last_heading_time = 0;
const float heading_alpha = 0.60; // Complementary filter weight (0.60 = 60% gyro, 40% mag) - strong mag correction to catch drift

// ============================================================================
// SETUP
// ============================================================================

void setup() {
  Serial.begin(115200);

  // Initialize motor pins
  pinMode(pwma, OUTPUT);
  pinMode(ain1, OUTPUT);
  pinMode(ain2, OUTPUT);
  pinMode(pwmb, OUTPUT);
  pinMode(bin1, OUTPUT);
  pinMode(bin2, OUTPUT);

  // Initialize servos
  gripperServo.attach(10);
  liftServo.attach(11);
  liftServo.write(30);
  gripperServo.write(10);

  // Initialize I2C communication
  Wire.begin(ARDUINO_I2C_ADDR);
  Wire.onReceive(receiveEvent);
  Wire.onRequest(requestEvent);

  // Initialize IMU
  bool lsm6ds_success, lis3mdl_success;
  lsm6ds_success = lsm6ds.begin_I2C();
  lis3mdl_success = lis3mdl.begin_I2C();
  
  if (!lsm6ds_success) {
    Serial.println("Failed to find LSM6DS chip");
  }
  if (!lis3mdl_success) {
    Serial.println("Failed to find LIS3MDL chip");
  }
  if (!(lsm6ds_success && lis3mdl_success)) {
    while (1) delay(10);
  }
  
  Serial.println("LSM6DS and LIS3MDL Found!");
  lsm6ds.setGyroRange(LSM6DS_GYRO_RANGE_500_DPS);
  
  // Calibrate magnetometer
  calibrateMagnetometer();
  last_heading_time = millis();

  lastHeartbeat = millis();
  Serial.println("System Ready.");
}

// ============================================================================
// MAIN LOOP
// ============================================================================

void loop() {
  // Heartbeat safety: stop motors if Pi communication lost
  if (millis() - lastHeartbeat > 1000) {
    stopMotors();
    return;
  }

  switch (currentCommand) {
    case 0:
      // Idle, no output to reduce clutter
      break;
      
    case 1: // FOLLOW LINE
      runPDLogic(i2cOffset, i2cDirection);
      break;

    case 2: // TURN LEFT
      if (consecutiveTurnCount >= turnThreshold) {
        Serial.println(F("CMD: TURN LEFT"));
        executeArc(200, 0.4, true, previousCommand);
        previousCommand = currentCommand;
        consecutiveTurnCount = 0;
        currentCommand = 0;
      } else {
        Serial.print(F("TURN LEFT queued: "));
        Serial.print(consecutiveTurnCount);
        Serial.print("/");
        Serial.println(turnThreshold);
      }
      break;

    case 3: // TURN RIGHT
      if (consecutiveTurnCount >= turnThreshold) {
        Serial.println(F("CMD: TURN RIGHT"));
        executeArc(200, 0.4, false, previousCommand);
        previousCommand = currentCommand;
        consecutiveTurnCount = 0;
        currentCommand = 0;
      } else {
        Serial.print(F("TURN RIGHT queued: "));
        Serial.print(consecutiveTurnCount);
        Serial.print("/");
        Serial.println(turnThreshold);
      }
      break;

    case 4: // GRAB SEQUENCE
      executeGrabSequence(grabXOffset, grabYOffset);
      break;

    case 5: // DROP SEQUENCE
      executeDropSequence();
      break;

    default:
      stopMotors();
      break;
  }
}


// ============================================================================
// I2C COMMUNICATION
// ============================================================================

void receiveEvent(int howMany) {
  lastHeartbeat = millis();
  int flush;

  Serial.print(F("I2C: "));
  Serial.print(howMany);
  Serial.println(F(" bytes"));

  // ---- 4-byte commands: Follow or Grab ----
  if (howMany == 4) {
    flush = Wire.read();
    int cmd = Wire.read();
    
    Serial.print(F("4-byte cmd: "));
    Serial.println(cmd);

    // If currently in grab sequence, only accept grab updates
    if (currentCommand == 4) {
      if (cmd == 4) {
        grabXOffset = (signed char)Wire.read();
        grabYOffset = (signed char)Wire.read();
      } else {
        Wire.read();
        Wire.read();
      }
      return;
    }

    // Handle follow command
    if (cmd == 1) {
      // Ignore follow commands during backup/camera re-acquisition period
      if (lastTurnBackupTime > 0 && (millis() - lastTurnBackupTime) < FOLLOW_IGNORE_TIME) {
        Serial.println(F("FOLLOW cmd ignored (backup settling period)"));
        Wire.read();
        Wire.read();
        return;
      }
      
      if (currentCommand != 1) {
        Serial.println(F("CMD: FOLLOW LINE"));
        lastError = 0;
      }
      if (consecutiveTurnCount > 0) {
        Serial.print(F("Turn buffer lost: "));
        Serial.print(consecutiveTurnCount);
        Serial.println(F(" counts cleared by FOLLOW"));
      }
      consecutiveTurnCount = 0;
      currentCommand = cmd;
      i2cOffset = Wire.read();
      i2cDirection = Wire.read();
      grabCommandCount = 0;
    }
    // Handle grab command
    else if (cmd == 4) {
      if (grabSequenceCompleted) {
        Wire.read();
        Wire.read();
        return;
      }

      if (currentCommand != 4) {
        grabCommandCount = 1;
      } else {
        grabCommandCount++;
      }

      grabXOffset = (signed char)Wire.read();
      grabYOffset = (signed char)Wire.read();

      if (grabCommandCount >= grabThreshold) {
        currentCommand = cmd;
      }
    }
    // Unknown 4-byte command
    else {
      Serial.print(F("Unknown 4-byte cmd: "));
      Serial.println(cmd);
      Wire.read();
      Wire.read();
      grabCommandCount = 0;
    }
  }
  // ---- 2-byte commands: Turns, Drop ----
  else if (howMany == 2) {
    flush = Wire.read();
    int cmd = Wire.read();
    
    Serial.print(F("2-byte cmd: "));
    Serial.println(cmd);

    // Ignore all non-grab commands during grab sequence
    if (currentCommand == 4) {
      return;
    }

    // Handle drop command
    if (cmd == 5) {
      if (!grabSequenceCompleted) {
        return;
      }

      if (dropSequenceCompleted) {
        return;
      }

      if (currentCommand != 5) {
        dropCommandCount = 1;
      } else {
        dropCommandCount++;
      }

      if (dropCommandCount >= dropThreshold) {
        currentCommand = cmd;
      }
    }
    // Handle other commands (turns)
    else {
      if (cmd == 2 || cmd == 3) {
        consecutiveTurnCount++;
        Serial.print(F("TURN CMD: ")); 
        Serial.println(cmd == 2 ? F("LEFT") : F("RIGHT"));
      } else {
        Serial.print(F("Unknown 2-byte cmd: "));
        Serial.println(cmd);
        if (consecutiveTurnCount > 0) {
          Serial.print(F("Turn buffer cleared: "));
          Serial.println(consecutiveTurnCount);
        }
        consecutiveTurnCount = 0;
      }

      grabCommandCount = 0;
      dropCommandCount = 0;
      currentCommand = cmd;
    }
  }
  else {
    Serial.print(F("Unexpected byte count: "));
    Serial.println(howMany);
  }
}

void requestEvent() {
  // Return status: 0 = ready, 1 = busy
  byte status = (currentCommand == 0 || currentCommand == 1) ? 0 : 1;
  Wire.write(status);
}


// ============================================================================
// LINE FOLLOWING
// ============================================================================

void runPDLogic(int offset, int dir) {
  float currentError = (dir == 1) ? offset : -offset;
  float derivative = currentError - lastError;
  int adjustment = (currentError * Kp) + (derivative * Kd);

  moveMotors(baseSpeed + adjustment, baseSpeed - adjustment);
  lastError = currentError;
}

// ============================================================================
// TURN EXECUTION
// ============================================================================

void executeArc(int outerSpeed, float ratio, bool isLeft, int prevCommand) {
  // Clear any leftover grab/drop state before turn
  grabPhase = 0;
  dropPhase = 0;
  grabSequenceCompleted = false;
  dropSequenceCompleted = false;
  
  // Determine settling time based on previous command
  // From line-follow: need full settling time (camera blinded by turn)
  // From backup/idle: minimal settling (already repositioned)
  int settleTime = (prevCommand == 1) ? TURN_SETTLE_FROM_FOLLOW : TURN_SETTLE_FROM_BACKUP;
  
  if (prevCommand == 1) {
    Serial.println(F("SETTLE: 5.75s (from line-follow)"));
  } else {
    Serial.println(F("SETTLE: 0.5s (from backup)"));
  }
  
  int innerSpeed = outerSpeed * ratio;
  float target_rotation = 90.0;  // Always rotate 90 degrees, measured relative to turn start
  float current_rotation = 0.0;  // Accumulated rotation since turn start
  unsigned long last_gyro_time = millis();
  
  Serial.print(F("TURN START: "));
  Serial.println(isLeft ? "LEFT" : "RIGHT");
  
  // Settling phase: move forward straight (duration depends on previous command)
  unsigned long settleStartTime = millis();
  while (millis() - settleStartTime < settleTime) {
    sensors_event_t accel, gyro, mag, temp;
    lsm6ds.getEvent(&accel, &gyro, &temp);
    
    // Move forward straight
    moveMotors(150, 150);
  }
  stopMotors();

  // Execute 90-degree turn using only gyro (relative rotation measurement)
  last_gyro_time = millis();
  current_rotation = 0.0;
  
  while (true) {
    sensors_event_t accel, gyro, mag, temp;
    lsm6ds.getEvent(&accel, &gyro, &temp);
    
    // Measure rotation using only gyro - no absolute heading reference
    unsigned long now = millis();
    float dt = (now - last_gyro_time) / 1000.0;
    last_gyro_time = now;
    
    // Integrate gyro Z to get relative rotation
    // Positive gyro_z = clockwise, negative = counterclockwise
    float rotation_this_frame = gyro.gyro.z * 57.2958 * dt;  // Convert rad/s to deg
    
    // For right turns, we expect to see negative gyro (counterclockwise)
    // For left turns, we expect positive gyro (clockwise)
    // Track absolute value of rotation
    current_rotation += abs(rotation_this_frame);
    
    float rotation_error = target_rotation - current_rotation;
    
    Serial.print(F("Rotation: "));
    Serial.print(current_rotation);
    Serial.print(F(" | Target: "));
    Serial.print(target_rotation);
    Serial.print(F(" | Error: "));
    Serial.println(rotation_error);

    // Check if we've rotated enough
    if (rotation_error < 1.5) {  // Tighter tolerance
      break;
    }

    // Speed damping: reduce speed as we approach target to prevent overshoot
    float speedMultiplier = 1.0;
    if (rotation_error < 8.0) {
      speedMultiplier = 0.3 + (0.7 * (rotation_error - 1.5) / 6.5);
    }
    
    int adjustedInnerSpeed = max(50, (int)(innerSpeed * speedMultiplier));
    int adjustedOuterSpeed = max(50, (int)(outerSpeed * speedMultiplier));

    if (isLeft) {
      moveMotors(adjustedInnerSpeed, adjustedOuterSpeed);
    } else {
      moveMotors(adjustedOuterSpeed, adjustedInnerSpeed);
    }
  }

  // Stop after turn
  stopMotors();
  delay(500);  // Brief settle time after turn

  // Backup straight to re-center tape in camera FOV
  Serial.println(F("BACKUP: Starting 5-second backup..."));
  unsigned long backupStartTime = millis();
  while (millis() - backupStartTime < TURN_BACKUP_TIME) {
    sensors_event_t accel, gyro, mag, temp;
    lsm6ds.getEvent(&accel, &gyro, &temp);
    getFusedHeading(gyro.gyro.z);  // Keep fused heading updated during backup
    
    moveMotors(-TURN_BACKUP_SPEED, -TURN_BACKUP_SPEED);
    
    // Debug: print backup progress every 2 seconds
    unsigned long elapsedBackup = millis() - backupStartTime;
    if (elapsedBackup % 2000 < 100) {
      Serial.print(F("BACKUP: "));
      Serial.print(elapsedBackup);
      Serial.println(F("ms elapsed"));
    }
  }
  stopMotors();
  Serial.println(F("BACKUP: Complete"));
  
  // Record backup completion time to ignore follow commands during camera re-acquisition
  lastTurnBackupTime = millis();
  
  // Reset state after turn and backup complete
  grabPhase = 0;
  dropPhase = 0;
  currentCommand = 0;
}


// ============================================================================
// GRAB SEQUENCE STATE MACHINE
// ============================================================================

void executeGrabSequence(int xOffset, int yOffset) {
  // Phase 0: Lateral centering + forward movement
  if (grabPhase == 0) {
    if (grabCommandCount == grabThreshold) {
      Serial.println(F("CMD: GRAB - Phase 0"));
    }
    float currentError = xOffset;
    float derivative = currentError - lastGrabError;
    int xAdjustment = (currentError * grabKp) + (derivative * grabKd);
    lastGrabError = currentError;

    int leftMotor = GRAB_BASE_SPEED + xAdjustment;
    int rightMotor = GRAB_BASE_SPEED - xAdjustment;

    moveMotors(leftMotor, rightMotor);

    if (abs(yOffset) <= 2) {
      Serial.println(F("CMD: GRAB - Phase 1"));
      grabPhase = 1;
      grabPhaseTime = millis();
    }
  }
  // Phase 1: Final push (2 seconds straight)
  else if (grabPhase == 1) {
    moveMotors(GRAB_BASE_SPEED, GRAB_BASE_SPEED);

    if (millis() - grabPhaseTime >= GRAB_FINAL_PUSH_TIME) {
      Serial.println(F("CMD: GRAB - Phase 2"));
      grabPhase = 2;
      stopMotors();
    }
  }
  // Phase 2: Execute gripper
  else if (grabPhase == 2) {
    Serial.println(F("CMD: GRAB - Phase 3 (Spin)"));
    executeGripper(true);
    grabPhase = 3;
  }
  // Phase 3: 180-degree spin turn
  else if (grabPhase == 3) {
    sensors_event_t accel, gyro, mag, temp;
    lsm6ds.getEvent(&accel, &gyro, &temp);
    
    // Update fused heading
    getFusedHeading(gyro.gyro.z);
    
    // Calculate heading change since start of phase 3
    // Note: we need to track the initial heading for phase 3
    // This is a simplified approach - store initial heading globally
    static float phase3_start_heading = 0;
    static bool phase3_initialized = false;
    
    if (!phase3_initialized) {
      phase3_start_heading = current_heading;
      phase3_initialized = true;
    }
    
    float heading_change = getHeadingChange(phase3_start_heading + 180);

    moveMotors(100, -100);  // Spin left

    if (abs(heading_change) < 5.0) {  // Within 5 degrees of 180 degrees
      Serial.println(F("CMD: GRAB - Complete"));
      stopMotors();
      phase3_initialized = false;  // Reset for next grab
      currentCommand = 0;
      grabSequenceCompleted = true;
    }
  }
}


// ============================================================================
// DROP SEQUENCE STATE MACHINE
// ============================================================================

void executeDropSequence() {
  // Phase 0: Initialize motor delay
  if (dropPhase == 0) {
    if (dropCommandCount == dropThreshold) {
      Serial.println(F("CMD: DROP - Phase 0"));
    }
    moveMotors(GRAB_BASE_SPEED, GRAB_BASE_SPEED);
    dropPhaseTime = millis();
    dropPhase = 1;
  }
  // Phase 1: Wait for motor delay to complete
  else if (dropPhase == 1) {
    if (millis() - dropPhaseTime >= DROP_MOTOR_DELAY_TIME) {
      Serial.println(F("CMD: DROP - Phase 1"));
      stopMotors();
      dropPhase = 2;
    } else {
      moveMotors(GRAB_BASE_SPEED, GRAB_BASE_SPEED);
    }
  }
  // Phase 2: Execute gripper drop
  else if (dropPhase == 2) {
    Serial.println(F("CMD: DROP - Complete"));
    executeGripper(false);
    currentCommand = 0;
    dropSequenceCompleted = true;
  }
}


// ============================================================================
// GRIPPER CONTROL
// ============================================================================

void executeGripper(bool grab) {
  if (grab) {
    liftServo.write(180);
    delay(1000);
    gripperServo.write(90);
    delay(1000);
    liftServo.write(90);
  } else {
    liftServo.write(180);
    delay(1000);
    gripperServo.write(0);
    delay(1000);
    liftServo.write(90);
  }
}

// ============================================================================
// MOTOR CONTROL
// ============================================================================

void moveMotors(int left, int right) {
  // Set left motor direction (positive = forward, negative = reverse)
  if (left >= 0) {
    digitalWrite(bin1, HIGH);
    digitalWrite(bin2, LOW);
  } else {
    digitalWrite(bin1, LOW);
    digitalWrite(bin2, HIGH);
  }
  int leftSpeed = abs(left);

  // Set right motor direction (positive = forward, negative = reverse)
  if (right >= 0) {
    digitalWrite(ain1, HIGH);
    digitalWrite(ain2, LOW);
  } else {
    digitalWrite(ain1, LOW);
    digitalWrite(ain2, HIGH);
  }
  int rightSpeed = abs(right);

  // Scale if either motor exceeds PWM max
  int maxVal = max(leftSpeed, rightSpeed);
  if (maxVal > 255) {
    float scale = 255.0 / maxVal;
    leftSpeed *= scale;
    rightSpeed *= scale;
  }

  // Apply scaled speeds
  analogWrite(pwma, constrain(rightSpeed, 0, 255));
  analogWrite(pwmb, constrain(leftSpeed, 0, 255));
}

void stopMotors() {
  digitalWrite(ain1, HIGH);
  digitalWrite(ain2, HIGH);
  digitalWrite(bin1, HIGH);
  digitalWrite(bin2, HIGH);
}

// ============================================================================
// MAGNETOMETER CALIBRATION (HARDCODED VALUES)
// ============================================================================

void calibrateMagnetometer() {
  // Hardcoded calibration values - obtained from initial calibration routine
  // To recalibrate: Use the calibration sketch, rotate robot in all directions,
  // and copy the printed offsets and scales here.
  
  mag_offset_x = -10.742474;
  mag_offset_y = -46.499561;
  mag_offset_z = -89.608306;
  
  mag_scale_x = 0.685544;
  mag_scale_y = 0.770123;
  mag_scale_z = 4.118421;
  
  // Calibration values loaded successfully
}

// ============================================================================
// HEADING FUSION (Gyro + Magnetometer)
// ============================================================================

float getFusedHeading(float gyro_z) {
  unsigned long now = millis();
  float dt = (now - last_heading_time) / 1000.0;
  last_heading_time = now;
  
  // Update heading from gyro (degrees per second to degrees)
  float gyro_heading = current_heading + (gyro_z * 57.2958) * dt;
  
  // Get magnetometer heading
  sensors_event_t mag;
  lis3mdl.getEvent(&mag);
  
  // Apply calibration to magnetometer data
  float mag_x = (mag.magnetic.x - mag_offset_x) * mag_scale_x;
  float mag_y = (mag.magnetic.y - mag_offset_y) * mag_scale_y;
  
  // Calculate heading from magnetometer (in degrees, 0-360)
  float mag_heading = atan2(mag_y, mag_x) * 57.2958; // Convert radians to degrees
  if (mag_heading < 0) mag_heading += 360;
  
  // Handle wrap-around when crossing 0 degrees
  float heading_diff = mag_heading - gyro_heading;
  if (heading_diff > 180) heading_diff -= 360;
  if (heading_diff < -180) heading_diff += 360;
  
  // Complementary filter: combine gyro (fast, drifts) with mag (accurate, noisy)
  current_heading = gyro_heading + (heading_diff * (1.0 - heading_alpha));
  
  // Normalize to 0-360 range
  if (current_heading < 0) current_heading += 360;
  if (current_heading >= 360) current_heading -= 360;
  
  return current_heading;
}

// ============================================================================
// HEADING CHANGE CALCULATOR
// ============================================================================

float getHeadingChange(float target_angle) {
  float diff = target_angle - current_heading;
  
  // Normalize to -180 to 180 range
  if (diff > 180) diff -= 360;
  if (diff < -180) diff += 360;
  
  return diff;
}