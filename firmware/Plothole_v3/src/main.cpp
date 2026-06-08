#include <Arduino.h>
#include <Arduino_BMI270_BMM150.h>

#include "led_controller.h"

void setup() {
  IMU.begin();
  beginLED();
}

void loop() {
  float ax, ay, az;

  if (IMU.accelerationAvailable()) {
    IMU.readAcceleration(ax, ay, az);
  }

  if (az < 1.2) {
     emitColor(0);
  } else if (az > 1.2 && az < 1.8) {
     emitColor(1);
  } else {
     emitColor(2);
  }
  
  delay(200);
}