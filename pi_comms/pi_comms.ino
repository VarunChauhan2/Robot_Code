#include <Wire.h>

// I2C address
#define I2C_ADDR 0x8

// Received data variable
volatile int receivedOffset = 0;
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
    
    // Print received offset to serial monitor
    Serial.print("[");
    Serial.print(receiveCount);
    Serial.print("] Received offset: ");
    Serial.println(receivedOffset);
  }
  
  delay(10);
}

// I2C receive callback function
void receiveEvent(int numBytes) {
  Serial.print("  > receiveEvent() triggered with ");
  Serial.print(numBytes);
  Serial.println(" bytes");
  
  // Read all available bytes
  while (Wire.available()) {
    receivedOffset = Wire.read();
    
    // Convert unsigned byte to signed integer
    if (receivedOffset > 127) {
      receivedOffset = receivedOffset - 256;
    }
    
    Serial.print("    Raw byte read, signed value: ");
    Serial.println(receivedOffset);
  }
  
  dataReceived = true;
}
