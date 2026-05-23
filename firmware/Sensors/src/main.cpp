#include <Arduino.h>
#include <Arduino_BMI270_BMM150.h>

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
  Serial.println("Ax Ay Az | Gx Gy Gz | Mx My Mz");
}

void loop() {
  float ax, ay, az;
  float gx, gy, gz;
  float mx, my, mz;

  if (IMU.accelerationAvailable()) {
    IMU.readAcceleration(ax, ay, az);
  }

  if (IMU.gyroscopeAvailable()) {
    IMU.readGyroscope(gx, gy, gz);
  }

  if (IMU.magneticFieldAvailable()) {
    IMU.readMagneticField(mx, my, mz);
  }
 
  char line[160];

  snprintf(line,sizeof(line),
    "A:%.3f,%.3f,%.3f | G:%.3f,%.3f,%.3f | M:%.3f,%.3f,%.3f",
    ax, ay, az,
    gx, gy, gz,
    mx, my, mz
  );

  Serial.println(line);
  
  delay(200);
}