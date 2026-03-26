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
// LINE FOLLOWING (PD CONTROL)
// ============================================================================

int Kp = 0.5;
float Kd = 0;
int lastError = 0;

// ============================================================================
// I2C & COMMUNICATION
// ============================================================================

const int ARDUINO_I2C_ADDR = 0x08;
volatile int currentCommand = 0;
volatile int i2cOffset = 0;
volatile int i2cDirection = 0;
unsigned long lastHeartbeat = 0;

// ============================================================================
// RETURN MODE SKIP TRACKING
// ============================================================================

bool in_return_mode = true;  // Start in return mode for this test
int return_turns_completed = 2;  // Pre-set to 2, so next turn will be skipped
bool skip_next_turn_in_return = true;  // Flag to skip the next turn (set from start)
bool in_return_turn_skip_recovery = false;
unsigned long return_turn_skip_recovery_start = 0;
const int RETURN_TURN_SKIP_RECOVERY_TIME = 10000;  // 10 seconds to traverse past back-to-back turns
bool in_return_continue_forward_until_turn = false;  // Continue moving forward after skip timeout

const int TURN_SETTLE_DEFAULT = 5500;  // Default settling time for turns
const float TURN_ANGLE_OFFSET_RIGHT = 2.5;
const float TURN_ANGLE_OFFSET_LEFT = -2.5;

int baseSpeed = 150;
int consecutiveTurnCount = 0;
const int turnThreshold = 10;

// ============================================================================
// GYRO CALIBRATION
// ============================================================================

float gyro_bias_x = 0, gyro_bias_y = 0, gyro_bias_z = 0;

// ============================================================================
// TEST STATE
// ============================================================================

int testState = 0;
// 0 = waiting for first turn (will be skipped)
// 1 = turn skip recovery period
// 2 = continue forward until turn
// 3 = complete

unsigned long stateStartTime = 0;

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
  bool lsm6ds_success;
  lsm6ds_success = lsm6ds.begin_I2C();
  
  if (!lsm6ds_success) {
    Serial.println("Failed to find LSM6DS chip");
    while (1) delay(10);
  }
  
  Serial.println("LSM6DS Found!");
  lsm6ds.setGyroRange(LSM6DS_GYRO_RANGE_500_DPS);
  
  calibrateGyro();

  Serial.println("System Ready.");
  Serial.println("========================================");
  Serial.println("Testing: TURN SKIP IN RETURN PHASE");
  Serial.println("========================================");
  Serial.println("Initial state:");
  Serial.println("- in_return_mode = true");
  Serial.println("- return_turns_completed = 2");
  Serial.println("- skip_next_turn_in_return = true");
  Serial.println("");
  Serial.println("Test sequence:");
  Serial.println("1. Detect first available turn");
  Serial.println("2. SKIP the turn (recovery activated)");
  Serial.println("3. 10-second recovery (move forward)");
  Serial.println("4. Continue forward listening for turns");
  Serial.println("5. Demonstrate where next turn would execute");
  Serial.println("========================================");
  Serial.println("Waiting for first turn detection...");
  
  testState = 0;
}

// ============================================================================
// MAIN LOOP
// ============================================================================

void loop() {
  // Heartbeat check - stop if no Pi communication
  if (millis() - lastHeartbeat > 1000) {
    stopMotors();
    return;
  }

  // Handle return turn skip recovery mode
  if (in_return_turn_skip_recovery) {
    if (millis() - return_turn_skip_recovery_start < RETURN_TURN_SKIP_RECOVERY_TIME) {
      // Still in recovery: move forward at base speed
      moveMotors(baseSpeed, baseSpeed);
      
      // Print progress every 2 seconds
      static unsigned long lastPrint = 0;
      if (millis() - lastPrint >= 2000) {
        unsigned long elapsed = millis() - return_turn_skip_recovery_start;
        Serial.print("[SKIP RECOVERY] In progress: ");
        Serial.print(elapsed);
        Serial.println(" ms");
        lastPrint = millis();
      }
      return;
    } else {
      // 10 second timeout reached: transition to continue-forward mode
      in_return_turn_skip_recovery = false;
      in_return_continue_forward_until_turn = true;
      Serial.println("[SKIP RECOVERY] Timeout reached!");
      Serial.println("[SKIP RECOVERY] Transitioning to continue-forward mode");
    }
  }

  // Handle continue forward until turn mode
  if (in_return_continue_forward_until_turn) {
    moveMotors(baseSpeed, baseSpeed);
  }

  switch (testState) {
    case 0:  // Waiting for first turn from Pi (will be skipped)
      // Handle line following
      if (currentCommand == 1) {  // LINE FOLLOW
        runPDLogic(i2cOffset, i2cDirection);
      }
      // Turn detection comes from Pi via I2C in receiveEvent
      if (currentCommand == 2 && consecutiveTurnCount >= turnThreshold) {
        Serial.print("[I2C] LEFT turn detected! (count: ");
        Serial.print(consecutiveTurnCount);
        Serial.println(")");
        Serial.println("[TEST] Checking skip conditions...");
        
        // Check if we should skip this turn (in return mode with skip flag set)
        if (in_return_mode && skip_next_turn_in_return) {
          Serial.println("[TEST] SKIPPING this turn! Entering recovery mode...");
          Serial.println("[TEST] - Reason: return_turns_completed >= 2 && skip_next_turn_in_return");
          in_return_turn_skip_recovery = true;
          return_turn_skip_recovery_start = millis();
          skip_next_turn_in_return = false;  // Clear flag after skipping
          consecutiveTurnCount = 0;
          currentCommand = 0;  // Clear command
          testState = 1;
        }
      }
      break;

    case 1:  // In skip recovery (handled at top of loop)
      if (!in_return_turn_skip_recovery) {
        // Recovery period complete
        Serial.println("[TEST] Recovery period complete!");
        Serial.println("[TEST] Now in continue-forward mode");
        Serial.println("[TEST] Robot will move forward and listen for turns");
        testState = 2;
        stateStartTime = millis();
      }
      break;

    case 2:  // Continue forward until turn
      if (millis() - stateStartTime >= 8000) {
        // After 8 seconds, transition to complete
        Serial.println("[TEST] Continue-forward mode running for 8 seconds");
        Serial.println("[TEST] In actual use, turns detected here would execute normally");
        
        in_return_continue_forward_until_turn = false;
        stopMotors();
        testState = 3;
      }
      break;

    case 3:  // Test complete
      static bool completed = false;
      if (!completed) {
        Serial.println("========================================");
        Serial.println("TEST COMPLETE: Turn Skip in Return Phase");
        Serial.println("========================================");
        Serial.println("Summary:");
        Serial.println("- Configured for return mode (pre-set)");
        Serial.println("- Detected first available turn");
        Serial.println("- SKIPPED the turn (skip flag was set)");
        Serial.println("- Entered 10-second skip recovery period");
        Serial.println("- Moved to continue-forward mode");
        Serial.println("========================================");
        completed = true;
      }
      moveMotors(0, 0);  // Stay stopped
      delay(100);
      break;
  }
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
// I2C COMMUNICATION
// ============================================================================

void receiveEvent(int howMany) {
  lastHeartbeat = millis();
  int flush;

  if (howMany == 4) {
    flush = Wire.read();
    int cmd = Wire.read();

    if (cmd == 1) {  // LINE FOLLOW
      if (currentCommand != 1) {
        lastError = 0;
      }
      consecutiveTurnCount = 0;
      currentCommand = cmd;
      i2cOffset = Wire.read();
      i2cDirection = Wire.read();
    } else {
      Wire.read();
      Wire.read();
    }
  } else if (howMany == 2) {
    flush = Wire.read();
    int cmd = Wire.read();

    if (cmd == 2) {  // LEFT turn only (return mode only accepts left turns)
      consecutiveTurnCount++;
      currentCommand = cmd;
    }
  }
}

void requestEvent() {
  byte status = (currentCommand == 0) ? 0 : 1;
  Wire.write(status);
}

// ============================================================================
// TURN EXECUTION (from controls.ino)
// ============================================================================

void executeArc(int outerSpeed, float ratio, bool isLeft) {
  int settleTime = TURN_SETTLE_DEFAULT;
  int innerSpeed = outerSpeed * ratio;
  float angleOffset = isLeft ? TURN_ANGLE_OFFSET_LEFT : TURN_ANGLE_OFFSET_RIGHT;
  float target_rotation = 90.0 + angleOffset;
  float current_rotation = 0.0;
  unsigned long last_gyro_time = millis();
  
  // Settle phase
  Serial.println("    [TURN] Settling phase...");
  unsigned long settleStartTime = millis();
  while (millis() - settleStartTime < settleTime) {
    sensors_event_t accel, gyro, temp;
    lsm6ds.getEvent(&accel, &gyro, &temp);
    moveMotorsStraight(150, false);
  }
  stopMotors();
  
  Serial.println("    [TURN] Rotation phase...");

  last_gyro_time = millis();
  current_rotation = 0.0;
  
  while (true) {
    sensors_event_t accel, gyro, temp;
    lsm6ds.getEvent(&accel, &gyro, &temp);
    
    unsigned long now = millis();
    float dt = (now - last_gyro_time) / 1000.0;
    last_gyro_time = now;
    
    float rotation_this_frame = (gyro.gyro.z - gyro_bias_z) * 57.2958 * dt;
    current_rotation += abs(rotation_this_frame);
    
    float rotation_error = target_rotation - current_rotation;

    if (rotation_error < 1.5) {
      break;
    }

    float speedMultiplier = 1.0;
    if (rotation_error < 8.0) {
      speedMultiplier = 0.3 + (0.7 * (rotation_error - 1.5) / 6.5);
    }
    
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

  stopMotors();
  delay(500);
  Serial.println("    [TURN] Turn complete");
}

// ============================================================================
// MOTOR CONTROL
// ============================================================================

void moveMotors(int left, int right) {
  if (left >= 0) {
    digitalWrite(bin1, HIGH);
    digitalWrite(bin2, LOW);
  } else {
    digitalWrite(bin1, LOW);
    digitalWrite(bin2, HIGH);
  }
  int leftSpeed = abs(left);

  if (right >= 0) {
    digitalWrite(ain1, HIGH);
    digitalWrite(ain2, LOW);
  } else {
    digitalWrite(ain1, LOW);
    digitalWrite(ain2, HIGH);
  }
  int rightSpeed = abs(right);

  analogWrite(pwmb, leftSpeed);
  analogWrite(pwma, rightSpeed);
}

void moveMotorsStraight(int speed, bool forward) {
  if (forward) {
    digitalWrite(bin1, HIGH);
    digitalWrite(bin2, LOW);
    digitalWrite(ain1, HIGH);
    digitalWrite(ain2, LOW);
  } else {
    digitalWrite(bin1, LOW);
    digitalWrite(bin2, HIGH);
    digitalWrite(ain1, LOW);
    digitalWrite(ain2, HIGH);
  }
  analogWrite(pwmb, speed);
  analogWrite(pwma, speed);
}

void stopMotors() {
  analogWrite(pwma, 0);
  analogWrite(pwmb, 0);
  digitalWrite(ain1, LOW);
  digitalWrite(ain2, LOW);
  digitalWrite(bin1, LOW);
  digitalWrite(bin2, LOW);
}

// ============================================================================
// GYRO CALIBRATION
// ============================================================================

void calibrateGyro() {
  gyro_bias_x = 0;
  gyro_bias_y = 0;
  gyro_bias_z = 0;

  Serial.println("Calibrating gyro. Please keep robot still...");
  delay(1000);

  int samples = 100;
  for (int i = 0; i < samples; i++) {
    sensors_event_t accel, gyro, temp;
    lsm6ds.getEvent(&accel, &gyro, &temp);

    gyro_bias_x += gyro.gyro.x;
    gyro_bias_y += gyro.gyro.y;
    gyro_bias_z += gyro.gyro.z;

    delay(10);
  }

  gyro_bias_x /= samples;
  gyro_bias_y /= samples;
  gyro_bias_z /= samples;

  Serial.print("Gyro bias X: ");
  Serial.print(gyro_bias_x);
  Serial.print(" Y: ");
  Serial.print(gyro_bias_y);
  Serial.print(" Z: ");
  Serial.println(gyro_bias_z);
  Serial.println("Gyro calibration complete!");
}
