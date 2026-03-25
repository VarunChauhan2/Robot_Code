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
const int TURN_SETTLE_DEFAULT = 7700;
const float TURN_ANGLE_OFFSET_RIGHT = 2.5;
const float TURN_ANGLE_OFFSET_LEFT = -2.5;

// ============================================================================
// DROP SEQUENCE
// ============================================================================

int dropPhase = 0;
unsigned long dropPhaseTime = 0;
int dropCommandCount = 0;
const int DROP_MOTOR_DELAY_TIME = 5000;
const int dropThreshold = 5;
bool dropSequenceCompleted = false;

// ============================================================================
// MAGNETOMETER CALIBRATION & FUSION
// ============================================================================

float mag_offset_x = 0, mag_offset_y = 0, mag_offset_z = 0;
float mag_scale_x = 1, mag_scale_y = 1, mag_scale_z = 1;

float gyro_bias_x = 0, gyro_bias_y = 0, gyro_bias_z = 0;

float current_heading = 0;
unsigned long last_heading_time = 0;
const float heading_alpha = 0.60;

// ============================================================================
// FUNCTION DECLARATIONS
// ============================================================================

void executeArc(int outerSpeed, float ratio, bool isLeft);
void executeDropSequence();

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
  
  calibrateGyro();
  
  last_heading_time = millis();
  lastHeartbeat = millis();
  Serial.println("System Ready.");
  
  // Auto-pickup: grip immediately without moving
  delay(500);
  executeGripper(true);
  delay(1000);
  Serial.println("Auto-grip complete. Ready for drop test.");
}

// ============================================================================
// MAIN LOOP
// ============================================================================

void loop() {
  if (millis() - lastHeartbeat > 1000) {
    stopMotors();
    return;
  }

  switch (currentCommand) {
    case 0:
      break;
      
    case 1: // FOLLOW LINE
      runPDLogic(i2cOffset, i2cDirection);
      break;

    case 2: // TURN LEFT
      if (consecutiveTurnCount >= turnThreshold) {
        executeArc(200, 0.1, true);
        consecutiveTurnCount = 0;
        currentCommand = 0;
      }
      break;

    case 3: // TURN RIGHT
      if (consecutiveTurnCount >= turnThreshold) {
        executeArc(200, 0.1, false);
        consecutiveTurnCount = 0;
        currentCommand = 0;
      }
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

  if (howMany == 4) {
    flush = Wire.read();
    int cmd = Wire.read();

    if (cmd == 1) {
      if (currentCommand != 1) {
        lastError = 0;
      }
      consecutiveTurnCount = 0;
      currentCommand = cmd;
      i2cOffset = Wire.read();
      i2cDirection = Wire.read();
    }
    else {
      Wire.read();
      Wire.read();
    }
  }
  else if (howMany == 2) {
    flush = Wire.read();
    int cmd = Wire.read();

    if (cmd == 5) {
      Serial.println("[I2C] Drop command received");
      if (dropSequenceCompleted) {
        return;
      }

      dropCommandCount++;

      Serial.print("[I2C] Drop count: ");
      Serial.println(dropCommandCount);

      if (dropCommandCount >= dropThreshold) {
        currentCommand = cmd;
      }
    }
    else if (cmd == 2 || cmd == 3) {
      consecutiveTurnCount++;
      currentCommand = cmd;
    }
  }
}

void requestEvent() {
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
  int settleTime = TURN_SETTLE_DEFAULT;
  int innerSpeed = outerSpeed * ratio;
  float angleOffset = isLeft ? TURN_ANGLE_OFFSET_LEFT : TURN_ANGLE_OFFSET_RIGHT;
  float target_rotation = 90.0 + angleOffset;
  float current_rotation = 0.0;
  unsigned long last_gyro_time = millis();
  
  unsigned long settleStartTime = millis();
  while (millis() - settleStartTime < settleTime) {
    sensors_event_t accel, gyro, mag, temp;
    lsm6ds.getEvent(&accel, &gyro, &temp);
    moveMotorsStraight(150, false);
  }
  stopMotors();

  last_gyro_time = millis();
  current_rotation = 0.0;
  
  while (true) {
    sensors_event_t accel, gyro, mag, temp;
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
  currentCommand = 0;
}

// ============================================================================
// DROP SEQUENCE STATE MACHINE
// ============================================================================

void executeDropSequence() {
  if (dropPhase == 0) {
    Serial.println("[DROP] Drop sequence started - Phase 0 (Motor forward)");
    moveMotors(160, 160);
    dropPhaseTime = millis();
    dropPhase = 1;
  }
  else if (dropPhase == 1) {
    if (millis() - dropPhaseTime >= DROP_MOTOR_DELAY_TIME) {
      Serial.println("[DROP] Phase 1 complete - entering Phase 2 (Gripper drop)");
      stopMotors();
      dropPhase = 2;
    } else {
      moveMotors(160, 160);
    }
  }
  else if (dropPhase == 2) {
    Serial.println("[DROP] Phase 2 - executing gripper drop");
    executeGripper(false);
    Serial.println("[DROP] Drop sequence complete!");
    
    stopMotors();
    delay(1000);
    
    currentCommand = 0;
    dropSequenceCompleted = true;
    dropPhase = 0;
  }
}

// ============================================================================
// GRIPPER CONTROL
// ============================================================================

void executeGripper(bool grab) {
  if (grab) {
    gripperServo.write(100);
    delay(500);
    liftServo.write(60);
  } else {
    liftServo.write(30);
    delay(500);
    gripperServo.write(10);
  }
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

  int maxVal = max(leftSpeed, rightSpeed);
  if (maxVal > 255) {
    float scale = 255.0 / maxVal;
    leftSpeed *= scale;
    rightSpeed *= scale;
  }

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
// GYRO CALIBRATION
// ============================================================================

void calibrateGyro() {
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
    
    delay(10);
  }
  
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
// HEADING CHANGE CALCULATOR
// ============================================================================

float getHeadingChange(float target_angle) {
  float diff = target_angle - current_heading;
  
  if (diff > 180) diff -= 360;
  if (diff < -180) diff += 360;
  
  return diff;
}

void moveMotorsStraight(int speed, bool backward) {
  sensors_event_t accel, gyro, temp;
  lsm6ds.getEvent(&accel, &gyro, &temp);
  
  float correctionFactor = 1.0 + ((gyro.gyro.z - gyro_bias_z) * 0.005);
  
  int leftSpeed = speed * correctionFactor;
  int rightSpeed = speed / correctionFactor;
  
  moveMotors(backward ? -leftSpeed : leftSpeed, 
             backward ? -rightSpeed : rightSpeed);
}
