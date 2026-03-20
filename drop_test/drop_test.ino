// ============================================================================
// DROP TEST SKETCH
// ============================================================================
// Tests drop operation: grip, move forward, then lower and release

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
Servo grip;  // Gripper servo (Pin 10)
Servo lift;  // Lift servo (Pin 11)

// ============================================================================
// TEST VARIABLES - EDIT THESE TO CHANGE TEST BEHAVIOR
// ============================================================================

int testSpeed = 150;             // Speed to move forward (0-255)
unsigned long testDuration = 1900; // Duration in milliseconds (ms)
unsigned long testStartTime = 0;
bool testStarted = false;

// ============================================================================
// SETUP
// ============================================================================

void setup() {
  Serial.begin(9600);
  Serial.println("\n\n=== DROP TEST ===");
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
  grip.attach(10);
  lift.attach(11);
  grip.write(10);    // Gripper open
  lift.write(30);    // Lift down
  
  // Start with grip
  Serial.println("\n[INIT] Gripping...");
  delay(1000);
  executeGripper(true);  // Close gripper and raise lift
}

// ============================================================================
// MAIN LOOP
// ============================================================================

void loop() {
  // Start forward movement on first iteration
  if (!testStarted) {
    Serial.println("\n[START] Moving forward with object...");
    testStartTime = millis();
    testStarted = true;
    moveMotors(testSpeed, testSpeed);
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
    
    // Execute drop sequence
    delay(500);
    executeGripper(false);  // Lower lift and open gripper
    
    // Loop forever after test completes
    while(1) {
      delay(1000);
    }
  }
}

// ============================================================================
// MOTOR CONTROL FUNCTIONS
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
  digitalWrite(ain1, LOW);
  digitalWrite(ain2, LOW);
  digitalWrite(bin1, LOW);
  digitalWrite(bin2, LOW);
  analogWrite(pwma, 0);
  analogWrite(pwmb, 0);
}

// ============================================================================
// SERVO CONTROL FUNCTIONS
// ============================================================================

void executeGripper(bool grab) {
  if (grab) {
    // Grab: close gripper and raise lift
    grip.write(100);  // Close gripper
    delay(500);
    lift.write(120);  // Raise lift
    Serial.println("[GRABBED] Gripper closed and lift raised");
  } else {
    // Drop: lower lift and open gripper
    lift.write(30);   // Lower lift
    delay(500);
    grip.write(10);   // Open gripper
    Serial.println("[DROPPED] Lift lowered and gripper opened");
  }
}
