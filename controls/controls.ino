#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Servo.h>

// Motor A (Right)
int pwma = 6; int ain2 = 7; int ain1 = 8;
// Motor B (Left)
int pwmb = 5; int bin1 = 2; int bin2 = 3;
// Servos 
Servo gripperServo; // Pin 10
Servo liftServo;    // Pin 11

// IMU
Adafruit_MPU6050 mpu;

const int ARDUINO_I2C_ADDR = 0x08; // Arduino I2C Address
int baseSpeed = 150; // motor speed 
float Kp = 0.5; // Proportional control variable
float Kd = 0; // Derivative Control Variable
int lastError = 0; // error tracking variable
unsigned long lastHeartbeat = 0; // safety variable to check if instructions are being received from Pi

// Shared variables for I2C data
volatile int currentCommand = 0; // command from Pi based on CV information (state of the system)
volatile int i2cOffset = 0; // offset data from Pi
volatile int i2cDirection = 0; // direction of offset; 1 = left, 0 = right

void setup() {
  Serial.begin(115200);
  
  // Initialize Motor Pins
  pinMode(pwma, OUTPUT); pinMode(ain1, OUTPUT); pinMode(ain2, OUTPUT);
  pinMode(pwmb, OUTPUT); pinMode(bin1, OUTPUT); pinMode(bin2, OUTPUT);

  // Initialize Servos 
  gripperServo.attach(10);
  liftServo.attach(11); 

  liftServo.write(30);
  gripperServo.write(10);

  // Initialize I2C from Pi
  Wire.begin(ARDUINO_I2C_ADDR); // initialize I2C address of Arduino to be seen from Pi
  Wire.onReceive(receiveEvent); // receive command from Pi
  Wire.onRequest(requestEvent); // receive status request from Pi

  // Initialize MPU6050 
  if (!mpu.begin()) {
    Serial.println("Failed to find MPU6050 chip"); 
    while (1) delay(10); 
  }
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  
  lastHeartbeat = millis();
  Serial.println("System Ready.");
}

void loop() {
  // Stop motors if Pi hasn't sent data in 1 second
  if (millis() - lastHeartbeat > 1000) {
    stopMotors();
    return;
  }

  switch (currentCommand) {
    case 1: // FOLLOW LINE (PD CONTROL)
      runPDLogic(i2cOffset, i2cDirection);
      break;

    case 2: // ARC LEFT (90 DEGREE)
      executeArc(200, 0.4, true); // Outer speed 200, Ratio 0.4
      currentCommand = 1; // Return to following
      break;

    case 3: // ARC RIGHT (90 DEGREE)
      executeArc(200, 0.4, false);
      currentCommand = 1;
      break;

    case 4: // GRAB SEQUENCE
      stopMotors();
      executeGripper(true);
      currentCommand = 0;
      break;

    case 5: // DROP SEQUENCE
      stopMotors();
      executeGripper(false);
      currentCommand = 0;
      break;

    default:
      stopMotors();
      break;
  }
}

// I2C Communication Handler
void receiveEvent(int howMany) {
  lastHeartbeat = millis();
  int flush;

  // Follow command case
  if (howMany == 4) {
    flush = Wire.read();
    int cmd = Wire.read();
    // Reset error if switching into Follow Mode
    if (cmd == 1 && currentCommand != 1) lastError = 0;

    currentCommand = cmd;
    i2cOffset = Wire.read();
    i2cDirection = Wire.read();
  } else if (howMany == 2) {
    // Other commands
    flush = Wire.read();
    currentCommand = Wire.read();
  }
}

// Busy Flag for GRAB/ARC/DROP
void requestEvent() {
  // 0 = Ready for next task, 1 = Busy executing Grab/Arc
  byte status = (currentCommand == 0 || currentCommand == 1) ? 0 : 1;
  Wire.write(status); 
}

// Control & Movement Functions
void runPDLogic(int offset, int dir) {
  float currentError = (dir == 1) ? offset : -offset; // error value
  
  float derivative = currentError - lastError; // Derivative control value
  int adjustment = (currentError * Kp) + (derivative * Kd); // PD adjustment
  
  moveMotors(baseSpeed + adjustment, baseSpeed - adjustment);
  lastError = currentError;
}

// Turn Function
void executeArc(int outerSpeed, float ratio, bool isLeft) {
  int innerSpeed = outerSpeed * ratio;
  float angleZ = 0;
  unsigned long lastTime = millis();
  bool lineFound = false; // checker for turn optimization

  delay(6000); // temp delay before new mount

  while (abs(angleZ) < 90.0 && !lineFound) {
    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);
    
    unsigned long now = millis();
    float dt = (now - lastTime) / 1000.0;
    lastTime = now;
    angleZ += (g.gyro.z * 57.2958) * dt;

    if (isLeft) moveMotors(innerSpeed, outerSpeed);
    else moveMotors(outerSpeed, innerSpeed);

    // Early exit if CV finds the next line after 65 degrees
    if (abs(angleZ) > 65.0 && Wire.available()) {
      if (Wire.peek() == 1) lineFound = true;
    }
  }
  lastError = 0;
  stopMotors();
}

void executeGripper(bool grab) {
  if (grab) {
    liftServo.write(180); delay(1000); 
    gripperServo.write(90); delay(1000);
    liftServo.write(90); 
  } else {
    liftServo.write(180); delay(1000);
    gripperServo.write(0); delay(1000);
    liftServo.write(90);
  }
}

void moveMotors(int left, int right) {
  // Find if either motor exceeds 255
  int maxVal = max(abs(left), abs(right));
  
  if (maxVal > 255) {
    float scale = 255.0 / maxVal;
    left *= scale;
    right *= scale;
  }

  // 2. Now apply the scaled (and ratio-preserved) speeds
  digitalWrite(ain1, HIGH); digitalWrite(ain2, LOW);
  analogWrite(pwma, constrain(right, 0, 255));
  
  digitalWrite(bin1, HIGH); digitalWrite(bin2, LOW);
  analogWrite(pwmb, constrain(left, 0, 255));
}

void stopMotors() {
  digitalWrite(ain1, HIGH); digitalWrite(ain2, HIGH);
  digitalWrite(bin1, HIGH); digitalWrite(bin2, HIGH);
}