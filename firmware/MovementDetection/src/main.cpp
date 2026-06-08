#include <Arduino.h>
#include <ArduinoBLE.h>
#include <Arduino_BMI270_BMM150.h>

/*
 * MovementDetection firmware for the Arduino Nano 33 BLE Sense.
 *
 * Streams the on-board BMI270 IMU (accelerometer + gyroscope) over Bluetooth
 * LE at 50 Hz. The host (Webots `movement_supervisor` controller) uses the
 * data to drive a virtual robot whose motion mirrors how the user is moving /
 * tilting the physical chip.
 *
 * Wire format (12 bytes per notification, little-endian):
 *   int16_t ax, ay, az, gx, gy, gz
 *
 * Scaling on the wire (matches what the host expects):
 *   acceleration:  int16 / 16384.0 -> g       (full scale +-2 g)
 *   gyroscope:     int16 / 131.0   -> deg/s   (full scale +-250 deg/s)
 *
 * 12 bytes fits comfortably inside the default BLE ATT MTU (23 bytes), so we
 * do not need to depend on MTU negotiation succeeding.
 */

BLEService imuService("19B10010-E8F2-537E-4F6C-D104768A1214");

// Fixed-length 12-byte characteristic.
BLECharacteristic imuChar(
  "19B10011-E8F2-537E-4F6C-D104768A1214",
  BLERead | BLENotify,
  12,
  true
);

static const unsigned long SAMPLE_INTERVAL_MS = 20;  // 50 Hz
static unsigned long lastSample = 0;

void setup() {
  /*Serial.begin(9600);
  
  while (!Serial) {
    delay(10); 
  }*/

  if (!IMU.begin()) {
    //Serial.println("Failed to initialize IMU.");
    while (1) { delay(10); }
  }
  if (!BLE.begin()) {
    //Serial.println("Failed to initialize BLE.");
    while (1) { delay(10); }
  }

  BLE.setLocalName("MovementTwin");
  BLE.setAdvertisedService(imuService);
  imuService.addCharacteristic(imuChar);
  BLE.addService(imuService);

  BLE.advertise();
  //Serial.println("BLE advertising as MovementTwin");
}

void loop() {
  BLE.poll();

  if (millis() - lastSample < SAMPLE_INTERVAL_MS) return;
  lastSample = millis();

  if (!IMU.accelerationAvailable() || !IMU.gyroscopeAvailable()) return;

  float ax, ay, az, gx, gy, gz;
  IMU.readAcceleration(ax, ay, az);  // returns values in g
  IMU.readGyroscope(gx, gy, gz);     // returns values in deg/s

  int16_t packet[6];
  packet[0] = (int16_t)(ax * 16384.0f);
  packet[1] = (int16_t)(ay * 16384.0f);
  packet[2] = (int16_t)(az * 16384.0f);
  packet[3] = (int16_t)(gx * 131.0f);
  packet[4] = (int16_t)(gy * 131.0f);
  packet[5] = (int16_t)(gz * 131.0f);

  imuChar.writeValue((uint8_t*)packet, sizeof(packet));
  //Serial.print("Sent IMU data: ax="); Serial.print(ax, 3);
}
