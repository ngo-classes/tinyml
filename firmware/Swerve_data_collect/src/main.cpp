#include <Arduino.h>
#include <Arduino_BMI270_BMM150.h>

#define WINDOW_SIZE 8 
#define BUFFER_MASK (WINDOW_SIZE - 1)

float buffer[WINDOW_SIZE];
int head = 0;
float running_sum = 0.0;
float average = 0.0;

void setup() {
  Serial.begin(9600);
  while (!Serial) {
    ; // wait for serial monitor
  }

  if (!IMU.begin()) {
    Serial.println("Failed to initialize IMU!");
    while (1) {
      delay(1000);
    }
  }

  Serial.println("Serial ready. Initializing IMU...");
  Serial.println("IMU ready.");
  Serial.println("Ax");
}

void loop() {
  float ax, ay, az;

  if (IMU.accelerationAvailable()) {
    IMU.readAcceleration(ax, ay, az);
  }
 
  running_sum -= buffer[head];
  buffer[head] = ax;
  running_sum += ax;
  head = (head + 1) & BUFFER_MASK;
  average = running_sum / WINDOW_SIZE;

  char line[10];
  snprintf(line,sizeof(line),"%.3f", average);
  Serial.println(line);
  delay(10);
}