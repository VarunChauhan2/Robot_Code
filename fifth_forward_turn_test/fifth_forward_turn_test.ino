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
// TURN TRACKING
// ============================================================================

bool in_pickup_backup = false;   // Flag for backup after 5th turn to reveal pickup location
unsigned long pickup_backup_start = 0;  // Timestamp when pickup backup started
const int PICKUP_BACKUP_TIME = 2000;  // 2 seconds backup to reveal pickup location

const int TURN_SETTLE_DEFAULT = 5500;  // Default settling time for turns
const int TURN_SETTLE_TURN5 = 7700;    // Settling time for 5th forward turn (ms)
const float TURN_ANGLE_OFFSET_RIGHT = 2.5;
const float TURN_ANGLE_OFFSET_LEFT = -2.5;

int baseSpeed = 150;
int consecutiveTurnCount = 0;  // Track consecutive turn detections from Pi
const int turnThreshold = 10;   // Threshold for detecting a valid turn

// ============================================================================
// GYRO CALIBRATION
// ============================================================================

float gyro_bias_x = 0, gyro_bias_y = 0, gyro_bias_z = 0;

// ============================================================================
// TEST STATE
// ============================================================================

int testState = 0;
// 0 = waiting for first turn
// 1 = executing turn
// 2 = backup period
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
  Serial.println("Testing: 5TH FORWARD TURN + BACKUP");
  Serial.println("========================================");
  Serial.println("Test will detect first available turn,");
  Serial.println("execute with TURN5 params (settling:");
  Serial.println("7700ms, ratio: -0.5), then backup for");
  Serial.println("2 seconds to reveal pickup location.");
  Serial.println("Waiting for first turn detection...");
  Serial.println("========================================");
  
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

  // Check if in backup mode
  if (in_pickup_backup) {
    if (millis() - pickup_backup_start < PICKUP_BACKUP_TIME) {
      // Still backing up: move backward at base speed
      moveMotors(-baseSpeed, -baseSpeed);
      
      // Print progress every 500ms
      static unsigned long lastPrint = 0;
      if (millis() - lastPrint >= 500) {
        unsigned long elapsed = millis() - pickup_backup_start;
        Serial.print("[BACKUP] In progress: ");
        Serial.print(elapsed);
        Serial.println(" ms");
        lastPrint = millis();
      }
      return;
    } else {
      // Backup complete
      in_pickup_backup = false;
      stopMotors();
      Serial.println("[BACKUP] Complete!");
      testState = 3;
    }
  }

  switch (testState) {
  switch (testState) {
    case 0:  // Waiting for first turn from Pi
      if (currentCommand == 1) {  // LINE FOLLOW
        runPDLogic(i2cOffset, i2cDirection);
      }
      if (currentCommand == 2 && consecutiveTurnCount >= turnThreshold) {
        Serial.print("[I2C] LEFT turn detected! (count: ");
        Serial.print(consecutiveTurnCount);
        Serial.println(")");
        Serial.println("[TEST] Executing turn with TURN5 parameters...");
        Serial.println("  - Settle time: 7700ms");
        Serial.println("  - Ratio: -0.5");
        testState = 1;
      }
      break;

    case 1:  // Execute the turn
      // Execute turn with TURN5 parameters (different settling time and ratio)
      executeArc(200, -0.5, true, TURN_SETTLE_TURN5);
      consecutiveTurnCount = 0;
      Serial.println("[TEST] Turn executed!");
      
      // Trigger backup phase
      in_pickup_backup = true;
      pickup_backup_start = millis();
      Serial.println("[TEST] Entering 2-second backup phase...");
      Serial.println("[TEST] Purpose: Reveal pickup location for vision");
      testState = 2;
      break;

    case 2:  // Backup period (handled in if statement above)
      break;

    case 3:  // Test complete
      static bool completed = false;
      if (!completed) {
        Serial.println("========================================");
        Serial.println("TEST COMPLETE: 5th Forward Turn + Backup");
        Serial.println("========================================");
        completed = true;
      }
      moveMotors(0, 0);  // Stay stopped
      delay(100);
      break;
  }
}

// ============================================================================
// TURN EXECUTION (from controls.ino - with customSettleTime parameter)
// ============================================================================

void executeArc(int outerSpeed, float ratio, bool isLeft, int customSettleTime = 0) {
  int settleTime = (customSettleTime > 0) ? customSettleTime : TURN_SETTLE_DEFAULT;
  int innerSpeed = outerSpeed * ratio;
  float angleOffset = isLeft ? TURN_ANGLE_OFFSET_LEFT : TURN_ANGLE_OFFSET_RIGHT;
  float target_rotation = 90.0 + angleOffset;
  float current_rotation = 0.0;
  unsigned long last_gyro_time = millis();
  
  // Settle phase
  Serial.print("  [TURN] Settling phase (");
  Serial.print(settleTime);
  Serial.println(" ms)...");
  unsigned long settleStartTime = millis();
  while (millis() - settleStartTime < settleTime) {
    sensors_event_t accel, gyro, temp;
    lsm6ds.getEvent(&accel, &gyro, &temp);
    moveMotorsStraight(150, false);
  }
  stopMotors();
  
  Serial.println("  [TURN] Rotation phase...");

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
  Serial.println("  [TURN] Turn complete");
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

    if (cmd == 2) {  // LEFT turn only (5th turn is always left)
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
