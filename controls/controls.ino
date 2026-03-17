#include <Wire.h>
#include <Adafruit_MPU6050.h>
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
Adafruit_MPU6050 mpu;

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
const int TURN_BACKUP_TIME = 1000;    // 1 second backup duration
const int TURN_BACKUP_SPEED = 75;     // Half of baseSpeed (150)

// ============================================================================
// GRAB SEQUENCE
// ============================================================================

volatile int grabXOffset = 0;
volatile int grabYOffset = 0;
int grabPhase = 0;
unsigned long grabPhaseTime = 0;
float grabTurnAngle = 0;
unsigned long grabTurnLastTime = 0;
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
  if (!mpu.begin()) {
    Serial.println("Failed to find MPU6050 chip");
    while (1) delay(10);
  }
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);

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
    case 1: // FOLLOW LINE
      runPDLogic(i2cOffset, i2cDirection);
      break;

    case 2: // TURN LEFT
      if (consecutiveTurnCount >= turnThreshold) {
        executeArc(200, 0.4, true);
        consecutiveTurnCount = 0;
        currentCommand = 0;  // Wait for new command
      }
      break;

    case 3: // TURN RIGHT
      if (consecutiveTurnCount >= turnThreshold) {
        executeArc(200, 0.4, false);
        consecutiveTurnCount = 0;
        currentCommand = 0;  // Wait for new command
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

  // ---- 4-byte commands: Follow or Grab ----
  if (howMany == 4) {
    flush = Wire.read();
    int cmd = Wire.read();

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
      if (currentCommand != 1) lastError = 0;
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
      Wire.read();
      Wire.read();
      grabCommandCount = 0;
    }
  }
  // ---- 2-byte commands: Turns, Drop ----
  else if (howMany == 2) {
    flush = Wire.read();
    int cmd = Wire.read();

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
      } else {
        consecutiveTurnCount = 0;
      }

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

void executeArc(int outerSpeed, float ratio, bool isLeft) {
  int innerSpeed = outerSpeed * ratio;
  float angleZ = 0;
  unsigned long lastTime = millis();

  delay(6000);  // Camera mount settling time

  // Execute 90-degree turn
  while (abs(angleZ) < 90.0) {
    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);

    unsigned long now = millis();
    float dt = (now - lastTime) / 1000.0;
    lastTime = now;
    angleZ += (g.gyro.z * 57.2958) * dt;

    Serial.println(angleZ);

    if (isLeft) {
      moveMotors(innerSpeed, outerSpeed);
    } else {
      moveMotors(outerSpeed, innerSpeed);
    }
  }

  // Stop after turn
  lastError = 0;
  stopMotors();

  // Backup straight to re-center tape in camera FOV
  unsigned long backupStartTime = millis();
  while (millis() - backupStartTime < TURN_BACKUP_TIME) {
    moveMotors(-TURN_BACKUP_SPEED, -TURN_BACKUP_SPEED);
  }
  stopMotors();
}


// ============================================================================
// GRAB SEQUENCE STATE MACHINE
// ============================================================================

void executeGrabSequence(int xOffset, int yOffset) {
  // Phase 0: Lateral centering + forward movement
  if (grabPhase == 0) {
    float currentError = xOffset;
    float derivative = currentError - lastGrabError;
    int xAdjustment = (currentError * grabKp) + (derivative * grabKd);
    lastGrabError = currentError;

    int leftMotor = GRAB_BASE_SPEED + xAdjustment;
    int rightMotor = GRAB_BASE_SPEED - xAdjustment;

    moveMotors(leftMotor, rightMotor);

    if (abs(yOffset) <= 2) {
      grabPhase = 1;
      grabPhaseTime = millis();
    }
  }
  // Phase 1: Final push (2 seconds straight)
  else if (grabPhase == 1) {
    moveMotors(GRAB_BASE_SPEED, GRAB_BASE_SPEED);

    if (millis() - grabPhaseTime >= GRAB_FINAL_PUSH_TIME) {
      grabPhase = 2;
      stopMotors();
    }
  }
  // Phase 2: Execute gripper
  else if (grabPhase == 2) {
    executeGripper(true);
    grabPhase = 3;
    grabTurnAngle = 0;
    grabTurnLastTime = millis();
  }
  // Phase 3: 180-degree spin turn
  else if (grabPhase == 3) {
    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);

    unsigned long now = millis();
    float dt = (now - grabTurnLastTime) / 1000.0;
    grabTurnLastTime = now;
    grabTurnAngle += (g.gyro.z * 57.2958) * dt;

    moveMotors(100, -100);  // Spin left

    if (abs(grabTurnAngle) >= 180.0) {
      stopMotors();
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
    moveMotors(GRAB_BASE_SPEED, GRAB_BASE_SPEED);
    dropPhaseTime = millis();
    dropPhase = 1;
  }
  // Phase 1: Wait for motor delay to complete
  else if (dropPhase == 1) {
    if (millis() - dropPhaseTime >= DROP_MOTOR_DELAY_TIME) {
      stopMotors();
      dropPhase = 2;
    } else {
      moveMotors(GRAB_BASE_SPEED, GRAB_BASE_SPEED);
    }
  }
  // Phase 2: Execute gripper drop
  else if (dropPhase == 2) {
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