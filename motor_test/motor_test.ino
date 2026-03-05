#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>

int pwma = 6;
int ain2 = 7;
int ain1 = 8;
int pwmb = 5; 
int bin1 = 2; 
int bin2 = 3; 

Adafruit_MPU6050 mpu;

void setup() {
  Serial.begin(115200);

  pinMode(pwma, OUTPUT);
  pinMode(ain1, OUTPUT);
  pinMode(ain2, OUTPUT);
  pinMode(pwmb, OUTPUT);
  pinMode(bin1, OUTPUT);
  pinMode(bin2, OUTPUT);
  
  // Initialize MPU6050 
  if (!mpu.begin()) {
    Serial.println("Failed to find MPU6050 chip"); 
    while (1) delay(10); 
  }
  // No need to connect STBY if you want it always on, 
  // as it has an internal pull-up to Vcc[cite: 27, 82].
}

void loop() {
  int innerSpeed = 150 * 0.2;
  float angleZ = 0;
  unsigned long lastTime = millis();

  while (abs(angleZ) < 90.0) {
    /*sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);
    
    unsigned long now = millis();
    float dt = (now - lastTime) / 1000.0;
    lastTime = now;
    angleZ += (g.gyro.z * 57.2958) * dt;

    Serial.println(angleZ);

    moveMotors(innerSpeed, 150);*/

    moveMotors(100, 0);
    delay(500);
    moveMotors(0, 0);
    delay(500);
    moveMotors(0, 100);
    delay(500);
    moveMotors(0, 0);
    delay(500);
  }

  moveMotors(0, 0);

  delay(10000);
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