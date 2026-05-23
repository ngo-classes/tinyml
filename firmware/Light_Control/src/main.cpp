#include <Arduino.h>
#include <ArduinoBLE.h>

#include "led_controller.h"

BLEService tinymlService("19B10000-E8F2-537E-4F6C-D104768A1214");

BLEStringCharacteristic txChar(
  "19B10001-E8F2-537E-4F6C-D104768A1214",
  BLENotify,
  40
);

BLEStringCharacteristic rxChar(
  "19B10002-E8F2-537E-4F6C-D104768A1214",
  BLEWrite,
  40
);

unsigned long lastSend = 0;
int counter = 0;

void setup() {
  Serial.begin(9600);

  /* This while loop is necessary to ensure the Serial connection is established before we proceed with BLE setup. 
     On some boards, especially those with native USB, the Serial connection may not be ready immediately after begin() is called. 
     By waiting for Serial to be available, we can ensure that our debug prints will actually show up in the console.
  */
  while (!Serial) {
    delay(10); 
  }
  
  
  if (!BLE.begin()) {
    Serial.println("BLE failed to start.");
    while (1);
  }

  beginLED();
  setoff();

  BLE.setLocalName("TinyML-Nano33");
  BLE.setAdvertisedService(tinymlService);

  tinymlService.addCharacteristic(txChar);
  tinymlService.addCharacteristic(rxChar);
  BLE.addService(tinymlService);

  txChar.writeValue("ready");

  BLE.advertise();
  Serial.println("BLE advertising as TinyML-Nano33");
}

void loop() {
  BLEDevice central = BLE.central();

  if (central) {
    Serial.print("Connected to: ");
    Serial.println(central.address());

    while (central.connected()) {
      if (rxChar.written()) {
        String cmd = rxChar.value();

        Serial.print("Received: ");
        Serial.println(cmd);

        if (cmd == "red") {
          setred();
        } else if (cmd == "green") {
          setgreen();
        } else if (cmd == "yellow") {
          setyellow();
        } else if (cmd == "off") {
          setoff();
        }
      }

      if (millis() - lastSend > 1000) {
        lastSend = millis();

        String msg = "count=" + String(counter++);
        txChar.writeValue(msg);

        Serial.println(msg);
      }

      BLE.poll();
    }

    Serial.println("Disconnected.");
  }
}