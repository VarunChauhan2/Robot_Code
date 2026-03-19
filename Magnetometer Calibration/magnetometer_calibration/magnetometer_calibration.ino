// ============================================================================
// MAGNETOMETER CALIBRATION SKETCH (AUTO SPINNING)
// ============================================================================

#include <Wire.h>
#include <Adafruit_LSM6DSOX.h>
#include <Adafruit_LIS3MDL.h>
#include <Adafruit_Sensor.h>

Adafruit_LSM6DSOX lsm6ds;
Adafruit_LIS3MDL lis3mdl;

// Motor pins (same as controls.ino)
int pwma = 6;
int ain1 = 8;
int ain2 = 7;
int pwmb = 5;
int bin1 = 2;
int bin2 = 3;

void setup() {
  Serial.begin(115200);
  delay(1000);
  
  // Initialize motor pins
  pinMode(pwma, OUTPUT);
  pinMode(ain1, OUTPUT);
  pinMode(ain2, OUTPUT);
  pinMode(pwmb, OUTPUT);
  pinMode(bin1, OUTPUT);
  pinMode(bin2, OUTPUT);
  
  Serial.println(F("\n===== LIS3MDL MAGNETOMETER CALIBRATION =====\n"));
  
  // Initialize sensors
  if (!lsm6ds.begin_I2C()) {
    Serial.println(F("ERROR: LSM6DS not found!"));
    while (1) delay(10);
  }
  
  if (!lis3mdl.begin_I2C()) {
    Serial.println(F("ERROR: LIS3MDL not found!"));
    while (1) delay(10);
  }
  
  Serial.println(F("Sensors found!"));
  lsm6ds.setGyroRange(LSM6DS_GYRO_RANGE_500_DPS);
  
  delay(500);
  Serial.println(F("\nINSTRUCTIONS:"));
  Serial.println(F("1. Place robot on flat surface"));
  Serial.println(F("2. Robot will spin automatically"));
  Serial.println(F("3. Do NOT touch it!\n"));
  
  delay(3000);
  Serial.println(F("Starting in 3 seconds...\n"));
  delay(3000);
  
  runCalibration();
}

void loop() {
  delay(1000);
}

void spinRobot(int speed) {
  // Spin left motor backward, right motor forward (spin clockwise)
  digitalWrite(bin1, LOW);
  digitalWrite(bin2, HIGH);
  digitalWrite(ain1, HIGH);
  digitalWrite(ain2, LOW);
  
  analogWrite(pwmb, speed);
  analogWrite(pwma, speed);
}

void stopMotors() {
  digitalWrite(ain1, HIGH);
  digitalWrite(ain2, HIGH);
  digitalWrite(bin1, HIGH);
  digitalWrite(bin2, HIGH);
  analogWrite(pwma, 0);
  analogWrite(pwmb, 0);
}

void runCalibration() {
  Serial.println(F("CALIBRATING... Robot spinning!\n"));
  
  // Spin the robot
  spinRobot(150);
  
  float min_x = 1000, max_x = -1000;
  float min_y = 1000, max_y = -1000;
  float min_z = 1000, max_z = -1000;
  
  unsigned long cal_start = millis();
  unsigned long cal_duration = 25000;
  unsigned long last_print = millis();
  
  while (millis() - cal_start < cal_duration) {
    sensors_event_t mag;
    lis3mdl.getEvent(&mag);
    
    min_x = min(min_x, mag.magnetic.x);
    max_x = max(max_x, mag.magnetic.x);
    min_y = min(min_y, mag.magnetic.y);
    max_y = max(max_y, mag.magnetic.y);
    min_z = min(min_z, mag.magnetic.z);
    max_z = max(max_z, mag.magnetic.z);
    
    if (millis() - last_print > 2000) {
      unsigned long remaining = cal_duration - (millis() - cal_start);
      Serial.print(remaining / 1000);
      Serial.println(F("s remaining..."));
      last_print = millis();
    }
    
    delay(10);
  }
  
  // Stop motors
  stopMotors();
  
  // Calculate calibration values
  float mag_offset_x = (min_x + max_x) / 2.0;
  float mag_offset_y = (min_y + max_y) / 2.0;
  float mag_offset_z = (min_z + max_z) / 2.0;
  
  float range_x = (max_x - min_x) / 2.0;
  float range_y = (max_y - min_y) / 2.0;
  float range_z = (max_z - min_z) / 2.0;
  
  float avg_range = (range_x + range_y + range_z) / 3.0;
  
  float mag_scale_x = (range_x > 0) ? (avg_range / range_x) : 1.0;
  float mag_scale_y = (range_y > 0) ? (avg_range / range_y) : 1.0;
  float mag_scale_z = (range_z > 0) ? (avg_range / range_z) : 1.0;
  
  // Print results
  Serial.println(F("\n===== CALIBRATION COMPLETE =====\n"));
  Serial.println(F("Copy these into controls.ino:\n"));
  
  Serial.print(F("mag_offset_x = "));
  Serial.print(mag_offset_x, 6);
  Serial.println(";");
  
  Serial.print(F("mag_offset_y = "));
  Serial.print(mag_offset_y, 6);
  Serial.println(";");
  
  Serial.print(F("mag_offset_z = "));
  Serial.print(mag_offset_z, 6);
  Serial.println(";\n");
  
  Serial.print(F("mag_scale_x = "));
  Serial.print(mag_scale_x, 6);
  Serial.println(";");
  
  Serial.print(F("mag_scale_y = "));
  Serial.print(mag_scale_y, 6);
  Serial.println(";");
  
  Serial.print(F("mag_scale_z = "));
  Serial.print(mag_scale_z, 6);
  Serial.println(";\n");
  
  Serial.println(F("Done! Upload controls.ino now."));
}
