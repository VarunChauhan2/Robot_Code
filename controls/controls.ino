#include <Wire.h>
#include <Adafruit_LSM6DSOX.h>
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

// ============================================================================
// I2C & COMMUNICATION
// ============================================================================

const int ARDUINO_I2C_ADDR = 0x08;
volatile int currentCommand = 0;
volatile int i2cOffset = 0;
volatile int i2cDirection = 0;

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
const int TURN_SETTLE_DEFAULT = 5500;  // Default settling time for all turns except turn 5 (ms)
const int TURN_SETTLE_TURN5 = 7700;  // Reduced settling time for 5th forward turn (ms)
const float TURN_ANGLE_OFFSET_RIGHT = 2.5;  // Extra degrees for right turns (undershoot compensation)
const float TURN_ANGLE_OFFSET_LEFT = -2.5;  // Reduce degrees for left turns (overshoot compensation)

int forward_turns_executed = 1;  // Track number of turns in forward direction
bool in_forward_turn_recovery = false;  // Flag for recovery period after 2nd turn (forward)
unsigned long forward_turn_recovery_start = 0;  // Timestamp of forward recovery start
const int FORWARD_TURN_RECOVERY_TIME = 10000;  // 10 seconds: ignore Pi, move forward (after 2nd forward turn)

// PICKUP BACKUP (after 5th forward turn to let vision see pickup location)
bool in_pickup_backup = false;  // Flag for backup at pickup area (5th turn)
unsigned long pickup_backup_start = 0;  // Timestamp when pickup backup started
const int PICKUP_BACKUP_TIME = 2000;  // 2 seconds backup to reveal pickup location

// RETURN MODE (after grab)
bool in_return_mode = false;  // After grab, robot is returning
int return_turns_completed = 0;  // Counts fully executed turns during return phase
bool skip_next_turn_in_return = false;  // Skip the next fully executed turn in return mode
bool in_return_turn_skip_recovery = false;  // Skip turn recovery mode (ignore turn, move forward)
unsigned long return_turn_skip_recovery_start = 0;  // Timestamp when skip recovery started
const int RETURN_TURN_SKIP_RECOVERY_TIME = 10000;  // 10 seconds to traverse past back-to-back turns (return)
bool in_return_continue_forward_until_turn = false;  // Continue moving forward after skip recovery timeout, listen for left turn to exit

// ============================================================================
// GRAB SEQUENCE
// ============================================================================

volatile int grabXOffset = 0;
volatile int grabYOffset = 0;
int grabPhase = 0;
unsigned long grabPhaseTime = 0;
int lastGrabError = 0;

const int GRAB_BASE_SPEED = 80;
const int GRAB_FINAL_PUSH_TIME = 2500;
const int GRAB_X_THRESHOLD = 3;
const int GRAB_CONFIRM_THRESHOLD = 5;  // Number of consistent grab commands before executing
float grabKp = 0.5;
float grabKd = 0;

bool grabSequenceCompleted = false;

// ============================================================================
// DROP SEQUENCE
// ============================================================================

const int DROP_MOTOR_DELAY_TIME = 1000;

bool dropSequenceCompleted = false;

// ============================================================================
// GYRO CALIBRATION
// ============================================================================

float gyro_bias_x = 0, gyro_bias_y = 0, gyro_bias_z = 0;

// ============================================================================
// FUNCTION DECLARATIONS (Forward Declarations)
// ============================================================================

void executeArc(int outerSpeed, float ratio, bool isLeft, int customSettleTime = 0);

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
  bool lsm6ds_success;
  lsm6ds_success = lsm6ds.begin_I2C();
  
  if (!lsm6ds_success) {
    Serial.println("Failed to find LSM6DS chip");
    while (1) delay(10);
  }
  
  Serial.println("LSM6DS Found!");
  lsm6ds.setGyroRange(LSM6DS_GYRO_RANGE_500_DPS);
  
  // Calibrate gyro
  calibrateGyro();

  Serial.println("System Ready.");
}

// ============================================================================
// MAIN LOOP
// ============================================================================

void loop() {
  // Check if in forward turn recovery mode (ignore Pi commands, move forward after 2nd turn)
  if (in_forward_turn_recovery) {
    if (millis() - forward_turn_recovery_start < FORWARD_TURN_RECOVERY_TIME) {
      // Still in recovery: move forward at base speed
      moveMotors(baseSpeed, baseSpeed);
      return;
    } else {
      // Recovery period complete
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
      // 10 second timeout reached: transition to continue-forward mode
      // Robot will keep moving forward, but now listens for left turn commands
      in_return_turn_skip_recovery = false;
      in_return_continue_forward_until_turn = true;
      currentCommand = 0;
    }
  }

  // Check if continuing forward until a left turn command arrives (after skip recovery timeout)
  if (in_return_continue_forward_until_turn) {
    moveMotors(baseSpeed, baseSpeed);
    // If currentCommand is set to 2 (left turn) by receiveEvent, fall through to process it
    if (currentCommand != 2) {
      return;  // Keep moving forward
    }
    // Otherwise fall through to execute the left turn
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
          in_return_turn_skip_recovery = true;
          return_turn_skip_recovery_start = millis();
          skip_next_turn_in_return = false;  // Clear flag after skipping
          consecutiveTurnCount = 0;
          currentCommand = 0;
        } else {
          // Execute LEFT turn in all cases (forward mode or return mode)
          
          // Different parameters for 5th forward turn vs other turns
          if (!in_return_mode && forward_turns_executed == 5) {
            // 5th forward turn: reduced settling time and different ratio
            executeArc(200, -0.5, true, TURN_SETTLE_TURN5);
          } else {
            // All other turns: default parameters (uses default settling time)
            executeArc(200, 0.1, true);
          }
          
          consecutiveTurnCount = 0;
          currentCommand = 0;
          
          // Clear continue-forward mode if we were in it
          if (in_return_continue_forward_until_turn) {
            in_return_continue_forward_until_turn = false;
          }
          
          // Track turns in return mode
          if (in_return_mode) {
            return_turns_completed++;
            
            // After 2nd turn in return mode, set flag to skip next turn
            if (return_turns_completed == 2) {
              skip_next_turn_in_return = true;
            }
          } else {
            forward_turns_executed++;
          }
        }
      } else {
      }
      break;

    case 3: // TURN RIGHT
      if (consecutiveTurnCount >= turnThreshold) {
        // In return mode, ignore all RIGHT turns (only LEFT turns allowed)
        if (in_return_mode) {
          consecutiveTurnCount = 0;
          currentCommand = 0;
        } else {
          // In forward mode, execute RIGHT turns normally
          executeArc(200, 0.1, false);
          consecutiveTurnCount = 0;
          currentCommand = 0;
          forward_turns_executed++;
        }
      } else {
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
  int flush;

  // If in forward turn recovery mode, ignore all Pi commands
  if (in_forward_turn_recovery) {
    // Consume bytes but don't process
    while (Wire.available()) {
      Wire.read();
    }
    return;
  }

  // If in pickup backup mode, ignore all Pi commands
  if (in_pickup_backup) {
    // Consume bytes but don't process
    while (Wire.available()) {
      Wire.read();
    }
    return;
  }

  // If in return turn skip recovery mode, ignore all Pi commands
  if (in_return_turn_skip_recovery) {
    // Consume bytes but don't process
    while (Wire.available()) {
      Wire.read();
    }
    return;
  }

  // If waiting for turn to exit continue-forward mode, only accept left turn commands
  if (in_return_continue_forward_until_turn) {
    if (howMany == 2) {
      flush = Wire.read();  // consume register byte
      int cmd = Wire.read();
      
      if (cmd == 2) {
        // Left turn received - accumulate consecutive count normally
        consecutiveTurnCount++;
        currentCommand = cmd;
      } else {
        // Any other 2-byte command (right turn, etc.) - reset counter and ignore
        consecutiveTurnCount = 0;
      }
    } else {
      // Consume other command types but don't process
      while (Wire.available()) {
        Wire.read();
      }
    }
    return;
  }

  // ---- 4-byte commands: Follow or Grab ----
  if (howMany == 4) {
    flush = Wire.read();
    int cmd = Wire.read();
    int byte3 = (signed char)Wire.read();
    int byte4 = (signed char)Wire.read();

    // Handle follow command
    if (cmd == 1) {
      // Ignore follow commands ONLY during active grab Phase 0 (non-blocking PD feedback phase)
      if (currentCommand == 4 && grabPhase == 0 && !grabSequenceCompleted) {
        // Currently executing grab in Phase 0 - we're using vision feedback for positioning
        return;
      }
      
      if (currentCommand != 1) {
        lastError = 0;
      }
      consecutiveTurnCount = 0;
      // Reset grab/drop counters when switching to follow command (ensure true consecutiveness)
      static int grabCommandCount = 0;
      static int dropCommandCount = 0;
      grabCommandCount = 0;
      dropCommandCount = 0;
      currentCommand = cmd;
      i2cOffset = byte3;
      i2cDirection = byte4;
    }
    // Handle grab command - accumulate confirmations
    else if (cmd == 4) {
      if (grabSequenceCompleted) {
        // Grab already completed, ignore further grab commands
        return;
      }

      grabXOffset = byte3;
      grabYOffset = byte4;
      
      // Accumulate grab command confirmations
      static int grabCommandCount = 0;
      grabCommandCount++;

      if (grabCommandCount >= GRAB_CONFIRM_THRESHOLD) {
        currentCommand = cmd;
        grabCommandCount = 0;  // Reset counter for potential future grabs (if ever allowed)
      }
    }
  }
  // ---- 2-byte commands: Turns, Drop ----
  else if (howMany == 2) {
    flush = Wire.read();
    int cmd = Wire.read();

    // Handle drop command - accumulate confirmations
    if (cmd == 5) {
      if (!grabSequenceCompleted) {
        // Can't drop until grab is complete
        return;
      }

      if (dropSequenceCompleted) {
        // Drop already completed, ignore further drop commands
        return;
      }

      // Accumulate drop command confirmations
      static int dropCommandCount = 0;
      dropCommandCount++;

      if (dropCommandCount >= 5) {  // Threshold for drop
        currentCommand = cmd;
        dropCommandCount = 0;  // Reset counter
      }
    }
    // Handle other commands (turns)
    else {
      // Ignore turn commands ONLY during active grab Phase 0 (non-blocking PD feedback phase)
      if (currentCommand == 4 && grabPhase == 0 && !grabSequenceCompleted) {
        // Currently executing grab in Phase 0 - don't accept turns
        return;
      }
      
      if (cmd == 2 || cmd == 3) {
        consecutiveTurnCount++;
      } else {
        consecutiveTurnCount = 0;
      }
      
      // Reset grab/drop counters when receiving turn commands (ensure true consecutiveness)
      static int grabCommandCount = 0;
      static int dropCommandCount = 0;
      grabCommandCount = 0;
      dropCommandCount = 0;

      currentCommand = cmd;
    }
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

void executeArc(int outerSpeed, float ratio, bool isLeft, int customSettleTime = 0) {
  // No need to clear grab/drop state - these sequences are now fully blocking
  // and will not be interrupted by turns
  
  int settleTime = (customSettleTime > 0) ? customSettleTime : TURN_SETTLE_DEFAULT;
  
  int innerSpeed = outerSpeed * ratio;
  float angleOffset = isLeft ? TURN_ANGLE_OFFSET_LEFT : TURN_ANGLE_OFFSET_RIGHT;
  float target_rotation = 90.0 + angleOffset;  // Target rotation with direction-specific offset
  float current_rotation = 0.0;  // Accumulated rotation since turn start
  unsigned long last_gyro_time = millis();
  
  // Settling phase: move forward straight (duration depends on previous command)
  unsigned long settleStartTime = millis();
  while (millis() - settleStartTime < settleTime) {
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

    // Check if we've rotated enough
    if (rotation_error < 1.5) {  // Tighter tolerance
      break;
    }

    // Speed damping: reduce speed as we approach target to prevent overshoot
    float speedMultiplier = 1.0;
    if (rotation_error < 8.0) {
      speedMultiplier = 0.3 + (0.7 * (rotation_error - 1.5) / 6.5);
    }
    
    // Preserve sign when applying minimum speed constraint
    int tempInner = (int)(innerSpeed * speedMultiplier);
    int adjustedInnerSpeed = (tempInner < 0) ? -max(50, abs(tempInner)) : max(50, tempInner);
    
    int tempOuter = (int)(outerSpeed * speedMultiplier);
    int adjustedOuterSpeed = (tempOuter < 0) ? -max(50, abs(tempOuter)) : max(50, tempOuter);

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
    in_forward_turn_recovery = true;
    forward_turn_recovery_start = millis();
  }
  
  // Check if we just completed the 5th turn in forward mode (LEFT turn entering pickup) and need backup
  if (!in_return_mode && forward_turns_executed == 5 && isLeft) {
    in_pickup_backup = true;
    pickup_backup_start = millis();
  }
  
  // Return to main loop
  currentCommand = 0;
}


// ============================================================================
// GRAB SEQUENCE STATE MACHINE
// ============================================================================

void executeGrabSequence(int xOffset, int yOffset) {
  // ========================================================================
  // PHASE 0: Non-blocking state machine (continuous PD feedback from Pi)
  // ========================================================================
  if (grabPhase == 0) {
    // Initialize phase 0 timer on first entry
    if (grabPhaseTime == 0) {
      grabPhaseTime = millis();
      Serial.println("[GRAB] === PHASE 0 START ===");
    }

    float currentError = xOffset;
    float derivative = currentError - lastGrabError;
    int xAdjustment = (currentError * grabKp) + (derivative * grabKd);
    lastGrabError = currentError;

    int leftMotor = GRAB_BASE_SPEED + xAdjustment;
    int rightMotor = GRAB_BASE_SPEED - xAdjustment;

    moveMotors(leftMotor, rightMotor);

    // Transition to Phase 1 when centered or timeout reached
    unsigned long phase0_elapsed = millis() - grabPhaseTime;
    // Require BOTH X and Y to be centered before transitioning (angle + position)
    if (yOffset <= 2 || phase0_elapsed > 10000) {  // 10 second timeout
      Serial.print("[GRAB] === PHASE 0 END === Elapsed: ");
      Serial.print(phase0_elapsed);
      Serial.print("ms | Final offsets: X=");
      Serial.print(xOffset);
      Serial.print(" Y=");
      Serial.println(yOffset);
      
      if (phase0_elapsed > 10000) {
        Serial.println("[GRAB] Phase 0 timeout - proceeding to Phase 1");
      } else {
        Serial.println("[GRAB] Phase 0 centered - proceeding to Phase 1");
      }

      grabPhase = 1;
      grabPhaseTime = 0;  // Reset for next phase
      return;  // Return to main loop, blocking phases start next iteration
    }
    return;  // Phase 0 non-blocking: always return to main loop
  }

  // ========================================================================
  // PHASES 1-5: Blocking sequential execution (atomic, no interruption)
  // ========================================================================

  // Phase 1: Final push forward (2 seconds straight)
  if (grabPhase == 1) {
    Serial.println("[GRAB] === PHASE 1 START === (2.5 second final push)");
    unsigned long phase1_start = millis();
    unsigned long last_print = phase1_start;
    
    while (millis() - phase1_start < GRAB_FINAL_PUSH_TIME) {
      moveMotors(GRAB_BASE_SPEED, GRAB_BASE_SPEED);  // Simple raw motor drive, no gyro correction
      
      // Print progress every 500ms
      unsigned long now = millis();
      if (now - last_print >= 500) {
        Serial.print("[GRAB] Phase 1 progress: ");
        Serial.print(now - phase1_start);
        Serial.println("ms");
        last_print = now;
      }
    }
    
    stopMotors();
    unsigned long phase1_elapsed = millis() - phase1_start;
    Serial.print("[GRAB] === PHASE 1 END === Total: ");
    Serial.print(phase1_elapsed);
    Serial.println("ms");
    
    grabPhase = 2;
  }

  // Phase 2: Execute gripper (close gripper and raise lift)
  if (grabPhase == 2) {
    executeGripper(true);
    grabPhase = 3;
  }

  // Phase 3: Backup sequence (7 seconds)
  if (grabPhase == 3) {
    unsigned long phase3_start = millis();
    while (millis() - phase3_start < 7000) {
      moveMotors(-GRAB_BASE_SPEED, -GRAB_BASE_SPEED);  // Simple raw motor drive, no gyro correction
    }
    stopMotors();
    grabPhase = 4;
  }

  // Phase 4: Rotate 90 degrees left (in-place rotation using gyro)
  if (grabPhase == 4) {
    float phase4_rotation = 0.0;
    unsigned long phase4_gyro_time = millis();

    while (true) {
      sensors_event_t accel, gyro, mag, temp;
      lsm6ds.getEvent(&accel, &gyro, &temp);

      // Measure rotation using gyro
      unsigned long now = millis();
      float dt = (now - phase4_gyro_time) / 1000.0;
      phase4_gyro_time = now;

      // Integrate gyro Z to get rotation
      float rotation_this_frame = (gyro.gyro.z - gyro_bias_z) * 57.2958 * dt;
      phase4_rotation += abs(rotation_this_frame);

      float rotation_error = 90.0 - phase4_rotation;

      // Check if we've rotated 90 degrees
      if (rotation_error < 1.5) {
        break;
      }

      // Speed damping as we approach target rotation
      float speedMultiplier = 1.0;
      if (rotation_error < 8.0) {
        speedMultiplier = 0.3 + (0.7 * (rotation_error - 1.5) / 6.5);
      }

      int rotationSpeed = (int)(100 * speedMultiplier);
      rotationSpeed = max(50, rotationSpeed);

      moveMotors(-rotationSpeed, rotationSpeed);
    }

    stopMotors();
    grabPhase = 5;
  }

  // Phase 5: Cleanup and activate return mode
  if (grabPhase == 5) {
    grabSequenceCompleted = true;
    currentCommand = 0;
    grabPhase = 0;
    grabPhaseTime = 0;
    lastGrabError = 0;

    // Activate return mode: robot will now return to start
    in_return_mode = true;
    return_turns_completed = 0;
    skip_next_turn_in_return = false;
  }
}


// ============================================================================
// DROP SEQUENCE STATE MACHINE
// ============================================================================

void executeDropSequence() {
  moveMotors(160, 160);
  
  // Phase 1: Move forward for DROP_MOTOR_DELAY_TIME
  unsigned long dropStartTime = millis();
  while (millis() - dropStartTime < DROP_MOTOR_DELAY_TIME) {
    // Keep moving forward
  }
  
  stopMotors();
  
  // Phase 2: Execute gripper drop (lower lift and open gripper)
  executeGripper(false);
  
  // Phase 3: Move forward for 2 seconds after drop
  moveMotors(160, 160);
  unsigned long postDropStartTime = millis();
  while (millis() - postDropStartTime < 2000) {
    // Keep moving forward
  }
  
  stopMotors();
  
  dropSequenceCompleted = true;
  currentCommand = 0;
}


// ============================================================================
// GRIPPER CONTROL
// ============================================================================

void executeGripper(bool grab) {
  if (grab) {
    // Grab: close gripper and raise lift
    gripperServo.write(100);  // Close gripper
    delay(500);
    liftServo.write(60);  // Raise lift
  } else {
    // Drop: lower lift and open gripper
    liftServo.write(30);   // Lower lift
    delay(500);
    gripperServo.write(10);   // Open gripper
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
  // Cut PWM to 0 - this is the safe way to stop
  // Do NOT pull all direction pins HIGH as that shorts the H-bridge!
  analogWrite(pwma, 0);
  analogWrite(pwmb, 0);
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
// MOTOR CORRECTION
// ============================================================================

void moveMotorsStraight(int speed, bool backward) {
  // Move with gyro-guided correction to keep perfectly straight
  // If robot is rotating, compensate with motor speed adjustment
  
  sensors_event_t accel, gyro, temp;
  lsm6ds.getEvent(&accel, &gyro, &temp);
  
  // Get calibrated gyro reading
  float gyro_z_calibrated = gyro.gyro.z - gyro_bias_z;
  
  // Apply dead zone to filter out noise (ignore rotations < 0.3 dps)
  if (abs(gyro_z_calibrated) < 0.3) {
    gyro_z_calibrated = 0;
  }
  
  // Apply correction - INVERTED from before!
  // Positive gyro (clockwise/right veer): reduce left speed, increase right speed
  // Negative gyro (counterclockwise/left veer): increase left speed, reduce right speed
  float correctionFactor = 1.0 - (gyro_z_calibrated * 0.003);  // INVERTED sign
  
  // Clamp correction factor to prevent extreme adjustments
  correctionFactor = constrain(correctionFactor, 0.7, 1.3);
  
  int leftSpeed = (int)(speed * correctionFactor);
  int rightSpeed = (int)(speed / correctionFactor);
  
  moveMotors(backward ? -leftSpeed : leftSpeed, 
             backward ? -rightSpeed : rightSpeed);
}