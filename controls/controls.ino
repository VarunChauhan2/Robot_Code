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
volatile int i2cOffset = 0;
volatile int i2cDirection = 0;
unsigned long lastHeartbeat = 0;

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
const int TURN_SETTLE_FROM_FOLLOW = 7500;  // 7.5s settling after line-follow
const float TURN_ANGLE_OFFSET_RIGHT = 2.5;  // Extra degrees for right turns (undershoot compensation)
const float TURN_ANGLE_OFFSET_LEFT = -2.5;  // Reduce degrees for left turns (overshoot compensation)

int forward_turns_executed = 0;  // Track number of turns in forward direction
bool in_forward_turn_recovery = false;  // Flag for recovery period after 2nd turn (forward)
unsigned long forward_turn_recovery_start = 0;  // Timestamp of forward recovery start
const int FORWARD_TURN_RECOVERY_TIME = 10000;  // 10 seconds: ignore Pi, move forward (after 2nd forward turn)

// PICKUP BACKUP (after 5th forward turn to let vision see pickup location)
bool in_pickup_backup = false;  // Flag for backup at pickup area (5th turn)
unsigned long pickup_backup_start = 0;  // Timestamp when pickup backup started
const int PICKUP_BACKUP_TIME = 7000;  // 7 seconds backup to reveal pickup location

// RETURN MODE (after grab)
bool in_return_mode = false;  // After grab, robot is returning
int return_turns_completed = 0;  // Counts fully executed turns during return phase
bool skip_next_turn_in_return = false;  // Skip the next fully executed turn in return mode
bool in_return_turn_skip_recovery = false;  // Skip turn recovery mode (ignore turn, move forward)
unsigned long return_turn_skip_recovery_start = 0;  // Timestamp when skip recovery started
const int RETURN_TURN_SKIP_RECOVERY_TIME = 10000;  // 10 seconds to traverse past back-to-back turns (return)

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

// Gyro bias calibration
float gyro_bias_x = 0, gyro_bias_y = 0, gyro_bias_z = 0;

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

    //delay(500);
    //gripperServo.detach();
    //liftServo.detach();


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
  
  // Calibrate gyro
  calibrateGyro();
  
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

  // Check if in forward turn recovery mode (ignore Pi commands, move forward after 2nd turn)
  if (in_forward_turn_recovery) {
    if (millis() - forward_turn_recovery_start < FORWARD_TURN_RECOVERY_TIME) {
      // Still in recovery: move forward at base speed
      moveMotors(baseSpeed, baseSpeed);
      return;
    } else {
      // Recovery period complete
      Serial.println(F("Forward turn recovery complete, resuming normal operation"));
      in_forward_turn_recovery = false;
      currentCommand = 0;
    }
  }

  // Check if in pickup backup mode (backup after 5th turn to reveal pickup location)
  if (in_pickup_backup) {
    if (millis() - pickup_backup_start < PICKUP_BACKUP_TIME) {
      // Still backing up: move backward at base speed
      moveMotors(-baseSpeed, -baseSpeed);
      return;
    } else {
      // Backup complete
      Serial.println(F("Pickup backup complete, ready for grab"));
      in_pickup_backup = false;
      currentCommand = 0;
    }
  }

  // Check if in return turn skip recovery mode (ignore a turn in return path, move forward)
  if (in_return_turn_skip_recovery) {
    if (millis() - return_turn_skip_recovery_start < RETURN_TURN_SKIP_RECOVERY_TIME) {
      // Still in recovery: move forward at base speed
      moveMotors(baseSpeed, baseSpeed);
      return;
    } else {
      // Recovery period complete
      Serial.println(F("Return turn skip recovery complete, resuming normal operation"));
      in_return_turn_skip_recovery = false;
      currentCommand = 0;
    }
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
        // Check if we should skip this turn (first back-to-back turn after 2nd turn in return)
        if (in_return_mode && skip_next_turn_in_return) {
          Serial.println(F("SKIPPING turn (LEFT) in return (back-to-back obstacle)"));
          in_return_turn_skip_recovery = true;
          return_turn_skip_recovery_start = millis();
          skip_next_turn_in_return = false;  // Clear flag after skipping
          consecutiveTurnCount = 0;
          currentCommand = 0;
        } else {
          // Execute LEFT turn in all cases (forward mode or return mode)
          Serial.println(F("CMD: TURN LEFT"));
          executeArc(200, -0.1, true);
          consecutiveTurnCount = 0;
          currentCommand = 0;
          
          // Track turns in return mode
          if (in_return_mode) {
            return_turns_completed++;
            Serial.print(F("Return turn #"));
            Serial.println(return_turns_completed);
            
            // After 2nd turn in return mode, set flag to skip next turn
            if (return_turns_completed == 2) {
              skip_next_turn_in_return = true;
              Serial.println(F("Will skip next turn"));
            }
          } else {
            forward_turns_executed++;
          }
        }
      } else {
        Serial.print(F("TURN LEFT queued: "));
        Serial.print(consecutiveTurnCount);
        Serial.print("/");
        Serial.println(turnThreshold);
      }
      break;

    case 3: // TURN RIGHT
      if (consecutiveTurnCount >= turnThreshold) {
        // In return mode, ignore all RIGHT turns (only LEFT turns allowed)
        if (in_return_mode) {
          Serial.println(F("IGNORING RIGHT turn in return mode (only LEFT turns allowed)"));
          consecutiveTurnCount = 0;
          currentCommand = 0;
        } else {
          // In forward mode, execute RIGHT turns normally
          Serial.println(F("CMD: TURN RIGHT"));
          executeArc(200, -0.1, false);
          consecutiveTurnCount = 0;
          currentCommand = 0;
          forward_turns_executed++;
        }
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

  // If in forward turn recovery mode, ignore all Pi commands
  if (in_forward_turn_recovery) {
    // Consume bytes but don't process
    while (Wire.available()) {
      Wire.read();
    }
    Serial.println(F("CMD ignored (forward turn recovery)"));
    return;
  }

  // If in pickup backup mode, ignore all Pi commands
  if (in_pickup_backup) {
    // Consume bytes but don't process
    while (Wire.available()) {
      Wire.read();
    }
    Serial.println(F("CMD ignored (pickup backup)"));
    return;
  }

  // If in return turn skip recovery mode, ignore all Pi commands
  if (in_return_turn_skip_recovery) {
    // Consume bytes but don't process
    while (Wire.available()) {
      Wire.read();
    }
    Serial.println(F("CMD ignored (return turn skip recovery)"));
    return;
  }

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

void executeArc(int outerSpeed, float ratio, bool isLeft) {
  // Clear any leftover grab/drop state before turn
  grabPhase = 0;
  dropPhase = 0;
  grabSequenceCompleted = false;
  dropSequenceCompleted = false;
  
  int settleTime = TURN_SETTLE_FROM_FOLLOW;
  
  int innerSpeed = outerSpeed * ratio;
  float angleOffset = isLeft ? TURN_ANGLE_OFFSET_LEFT : TURN_ANGLE_OFFSET_RIGHT;
  float target_rotation = 90.0 + angleOffset;  // Target rotation with direction-specific offset
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
    moveMotorsStraight(150, false);
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
    // Subtract gyro bias to get actual rotation
    float rotation_this_frame = (gyro.gyro.z - gyro_bias_z) * 57.2958 * dt;  // Convert rad/s to deg
    
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
  
  // Check if we just completed the 2nd turn in forward mode and need recovery
  if (!in_return_mode && forward_turns_executed == 2) {
    Serial.println(F("Entering forward turn recovery mode (10 seconds)"));
    in_forward_turn_recovery = true;
    forward_turn_recovery_start = millis();
  }
  
  // Check if we just completed the 5th turn in forward mode (LEFT turn entering pickup) and need backup
  if (!in_return_mode && forward_turns_executed == 5 && isLeft) {
    Serial.println(F("Entering pickup backup mode (7 seconds)"));
    in_pickup_backup = true;
    pickup_backup_start = millis();
  }
  
  // Reset state after turn complete
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
  // Phase 3: Backup sequence
  else if (grabPhase == 3) {
    Serial.println(F("CMD: GRAB - Phase 3 (Backup)"));
    moveMotors(-GRAB_BASE_SPEED, -GRAB_BASE_SPEED);  // Move backward
    grabPhaseTime = millis();
    grabPhase = 4;
  }
  
  // Phase 4: Wait for backup duration (7 seconds)
  else if (grabPhase == 4) {
    if (millis() - grabPhaseTime >= 7000) {  // 7 second backup
      Serial.println(F("CMD: GRAB - Phase 5 (Rotate 90° Left)"));
      grabPhase = 5;
      grabPhaseTime = millis();
    }
  }
  
  // Phase 5: Rotate 90 degrees left (in-place rotation)
  else if (grabPhase == 5) {
    sensors_event_t accel, gyro, mag, temp;
    lsm6ds.getEvent(&accel, &gyro, &temp);
    
    // Update fused heading
    getFusedHeading(gyro.gyro.z);
    
    // Calculate heading change since start of phase 5
    static float phase5_start_heading = 0;
    static bool phase5_initialized = false;
    
    if (!phase5_initialized) {
      phase5_start_heading = current_heading;
      phase5_initialized = true;
    }
    
    float heading_change = getHeadingChange(phase5_start_heading + 90);

    moveMotors(-100, 100);  // Rotate left: left motor backward, right motor forward

    if (abs(heading_change) < 2.0) {  // Within 2 degrees of 90 degrees
      Serial.println(F("CMD: GRAB - Complete"));
      stopMotors();
      phase5_initialized = false;  // Reset for next grab
      currentCommand = 0;
      grabSequenceCompleted = true;
      
      // Activate return mode: robot will now return to start with back-to-back turn handling
      in_return_mode = true;
      return_turns_completed = 0;  // Reset turn counter for return path
      skip_next_turn_in_return = false;  // Flag not set yet
      Serial.println(F("Return mode activated"));
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
    // Grab: close gripper and raise lift
    gripperServo.write(100);  // Close gripper
    delay(500);
    liftServo.write(120);  // Raise lift
    Serial.println("[GRABBED] Gripper closed and lift raised");
  } else {
    // Drop: lower lift and open gripper
    liftServo.write(30);   // Lower lift
    delay(500);
    gripperServo.write(10);   // Open gripper
    Serial.println("[DROPPED] Lift lowered and gripper opened");
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
  
  mag_offset_x = -6.452794;
  mag_offset_y = -48.063430;
  mag_offset_z = -95.184158;

  mag_scale_x = 0.730745;
  mag_scale_y = 0.794111;
  mag_scale_z = 2.686275;
  
  // Calibration values loaded successfully
}

// ============================================================================
// GYRO CALIBRATION
// ============================================================================

void calibrateGyro() {
  // Measure gyro bias while robot is stationary
  // Collect 100 samples and average them to get the bias offset
  
  Serial.println("Calibrating gyro... keep robot stationary!");
  delay(1000);
  
  float sum_x = 0, sum_y = 0, sum_z = 0;
  const int NUM_SAMPLES = 100;
  
  for (int i = 0; i < NUM_SAMPLES; i++) {
    sensors_event_t accel, gyro, temp;
    lsm6ds.getEvent(&accel, &gyro, &temp);
    
    sum_x += gyro.gyro.x;
    sum_y += gyro.gyro.y;
    sum_z += gyro.gyro.z;
    
    delay(10);  // Small delay between samples
  }
  
  // Calculate average bias
  gyro_bias_x = sum_x / NUM_SAMPLES;
  gyro_bias_y = sum_y / NUM_SAMPLES;
  gyro_bias_z = sum_z / NUM_SAMPLES;
  
  Serial.print("Gyro bias - X: ");
  Serial.print(gyro_bias_x);
  Serial.print(" Y: ");
  Serial.print(gyro_bias_y);
  Serial.print(" Z: ");
  Serial.println(gyro_bias_z);
  
  Serial.println("Gyro calibration complete!");
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

void moveMotorsStraight(int speed, bool backward) {
  // Move with gyro-guided correction to keep perfectly straight
  // If robot is rotating, compensate with motor speed adjustment
  
  sensors_event_t accel, gyro, temp;
  lsm6ds.getEvent(&accel, &gyro, &temp);
  
  // If rotating (gyro_z != 0), apply small correction
  // Subtract gyro bias for accurate correction
  float correctionFactor = 1.0 + ((gyro.gyro.z - gyro_bias_z) * 0.005);  // Scale calibrated gyro reading
  
  int leftSpeed = speed * correctionFactor;
  int rightSpeed = speed / correctionFactor;
  
  moveMotors(backward ? -leftSpeed : leftSpeed, 
             backward ? -rightSpeed : rightSpeed);
}