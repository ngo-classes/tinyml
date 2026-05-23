#include <Arduino.h>
#include <ArduinoBLE.h>
#include <Arduino_APDS9960.h>

#include "led_controller.h"

BLEService twinService("19B10000-E8F2-537E-4F6C-D104768A1214");

// 1. Proximity is a single byte (0-255)
BLEUnsignedCharCharacteristic proximityChar("19B10001-E8F2-537E-4F6C-D104768A1214", 
  BLERead | BLENotify);

// 2. Instructions are strings (Make sure to allocate max length buffer, e.g., 20 bytes)
BLEStringCharacteristic instructionChar("19B10002-E8F2-537E-4F6C-D104768A1214", 
  BLEWrite, 20);

unsigned long lastSample = 0;

void setup() {
  beginLED();
  setoff();

  Serial.begin(9600);

  /* This while loop is necessary to ensure the Serial connection is established before we proceed with BLE setup. 
     On some boards, especially those with native USB, the Serial connection may not be ready immediately after begin() is called. 
     By waiting for Serial to be available, we can ensure that our debug prints will actually show up in the console.
  */

  //while (!Serial) {
  //  delay(10); 
  //}

  if (!APDS.begin() || !BLE.begin()) {
    while (1); 
  }

  // --- ADD THESE ACCELERATOR PLUGINS TO BOOST SCAN DEPTH ---
  // boost_mode: a number, between 0 and 3, that specify the desired power increase. 
  // 0 sets boost to 100% (this is the default power value), 1 sets boost to 150%, 
  // 2 sets boost to 200% and 3 sets boost to 300%.
  APDS.setLEDBoost(0); 

  BLE.setLocalName("PhysicalTwin");
  BLE.setAdvertisedService(twinService);
  
  twinService.addCharacteristic(proximityChar);
  twinService.addCharacteristic(instructionChar); // Add your new string characteristic
  BLE.addService(twinService);
  
  BLE.advertise();
  Serial.println("BLE advertising as physical twin");
}

void loop() {
  BLE.poll(); 

  // Check if laptop sent an instruction string
  if (instructionChar.written()) {
    String command = instructionChar.value();
    command.trim();
    Serial.print("Received: ");
    Serial.println(command);
    
    if (command == "flash") {
      // Create a quick visible blink
      setred();  // Turn ON
      delay(300);                      // Brief blocking delay is okay for a simple indicator
      setoff(); // Turn OFF
    }
  }

  // Push distance samples up every 100ms
  if (millis() - lastSample > 100) {
    lastSample = millis();
    if (APDS.proximityAvailable()) {
      int prox = APDS.readProximity();
      proximityChar.writeValue(prox);
    }
  }
}