#include <Arduino.h>
#include <ArduinoBLE.h>
#include <Arduino_APDS9960.h>

BLEService twinService("19B10000-E8F2-537E-4F6C-D104768A1214");

// 1. Proximity is a single byte (0-255)
BLEStringCharacteristic proximityChar("19B10001-E8F2-537E-4F6C-D104768A1214", BLENotify, 40);

unsigned long lastSample = 0;

void setup() {

  Serial.begin(9600);
  while (!Serial) {
    delay(10); 
  }

  if (!APDS.begin() || !BLE.begin()) {
    while (1); 
  }

  BLE.setLocalName("PhysicalTwin");
  BLE.setAdvertisedService(twinService);
  
  twinService.addCharacteristic(proximityChar);
  BLE.addService(twinService);
  
  BLE.advertise();
  Serial.println("BLE advertising as physical twin");
}

void loop() {
  BLE.poll(); 

  if (millis() - lastSample > 500) {
    lastSample = millis();
    if (APDS.colorAvailable()) {
      int r, g, b, c;
      // read the color
      APDS.readColor(r, g, b, c);

      char line[40];
      snprintf(line,sizeof(line), "R:%d,G:%d,B:%d,C:%d", r, g, b, c);
      Serial.println(line);
      proximityChar.writeValue(line);
    }
  }
}