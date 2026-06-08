#include <Arduino.h>
#include <Arduino_BMI270_BMM150.h>

void setred() {
  digitalWrite(LEDR, LOW);
  digitalWrite(LEDG, HIGH);
  digitalWrite(LEDB, HIGH);
}

void setyellow() {
  digitalWrite(LEDR, LOW);
  digitalWrite(LEDG, LOW);
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

void emitColor(int color) {
    if (color == 0) {
        setgreen();
    } else if (color == 1) {
        setyellow();
    } else if (color == 2) {
        setred();
    }
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

  if (az < 1.2) {
     emitColor(0);
  } else if (az > 1.2 && az < 1.8) {
     emitColor(1);
  } else {
     emitColor(2);
  }
  
  delay(200);
}