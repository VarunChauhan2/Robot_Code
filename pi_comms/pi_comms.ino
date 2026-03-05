#include <Wire.h>

// I2C address
#define I2C_ADDR 0x8

// Received data variables
volatile byte receivedMode = 0;
volatile byte receivedOffset = 0;
volatile byte receivedDirection = 0;
volatile bool dataReceived = false;
volatile int receiveCount = 0;

void setup() {
  // Initialize serial communication
  Serial.begin(9600);
  delay(500);
  
  Serial.println("\n\n=== Arduino I2C Receiver Initializing ===");
  Serial.print("I2C Address: 0x");
  Serial.println(I2C_ADDR, HEX);
  
  // Initialize I2C as slave
  Wire.begin(I2C_ADDR);
  
  // Register receive callback
  Wire.onReceive(receiveEvent);
  
  Serial.println("Wire.begin() and onReceive() registered");
  Serial.println("Waiting for I2C data from Pi...\n");
}

void loop() {
  // Print heartbeat every 5 seconds
  static unsigned long lastHeartbeat = 0;
  if (millis() - lastHeartbeat > 5000) {
    lastHeartbeat = millis();
    Serial.print("Heartbeat - Receive count: ");
    Serial.println(receiveCount);
  }
  
  // Check if new data was received
  if (dataReceived) {
    dataReceived = false;
    receiveCount++;
    
    // Print received data to serial monitor
    Serial.print("[");
    Serial.print(receiveCount);
    Serial.print("] Mode: ");
    Serial.print(receivedMode);
    Serial.print(" | Offset: ");
    Serial.print(receivedOffset);
    Serial.print(" | Direction: ");
    Serial.println(receivedDirection == 1 ? "LEFT" : "RIGHT");
  }
  
  delay(10);
}

// I2C receive callback function
void receiveEvent(int numBytes) {
  Serial.print("  > receiveEvent() triggered with ");
  Serial.print(numBytes);
  Serial.println(" bytes");
  
  // Skip the register address byte (first byte from write_i2c_block_data)
  if (Wire.available()) {
    byte registerAddr = Wire.read();
    Serial.print("    Register: ");
    Serial.println(registerAddr);
  }
  
  // Read 3 bytes: mode, offset, direction
  if (Wire.available()) {
    receivedMode = Wire.read();
    Serial.print("    Byte 1 (Mode): ");
    Serial.println(receivedMode);
  }
  
  if (Wire.available()) {
    receivedOffset = Wire.read();
    Serial.print("    Byte 2 (Offset): ");
    Serial.println(receivedOffset);
  }
  
  if (Wire.available()) {
    receivedDirection = Wire.read();
    Serial.print("    Byte 3 (Direction): ");
    Serial.println(receivedDirection);
  }
  
  // Clear any remaining bytes
  while (Wire.available()) {
    Wire.read();
  }
  
  dataReceived = true;
}
