#include <Arduino.h>
#include <Arduino_BMI270_BMM150.h>

void setred() {
  digitalWrite(LEDR, LOW);
  digitalWrite(LEDG, HIGH);
  digitalWrite(LEDB, HIGH);
}

void setgreen() {
  digitalWrite(LEDR, HIGH);
  digitalWrite(LEDG, LOW);
  digitalWrite(LEDB, HIGH);
}

void beginLED() {
  pinMode(LEDR, OUTPUT);
  pinMode(LEDG, OUTPUT);
  pinMode(LEDB, OUTPUT);
}

void setup() {
  IMU.begin();
  beginLED();
}

void loop() {
  float ax, ay, az;

  if (IMU.accelerationAvailable()) {
    IMU.readAcceleration(ax, ay, az);
  }

  if (az > 1.5) {
     setred();
  } else {
     setgreen();
  }
  
  delay(200);
}