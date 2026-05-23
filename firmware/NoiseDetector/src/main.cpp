#include <Arduino.h>
#include <PDM.h>

#include "led_controller.h"
//#include "mic_controller.h"

short sampleBuffer[256]; 
volatile int samplesRead;

void onPDMdata() {
  int bytesAvailable = PDM.available();
  PDM.read(sampleBuffer, bytesAvailable);
  samplesRead = bytesAvailable / 2;
}

void setup() {
  // start the serial communication and wait for the port to open
  Serial.begin(9600); 
  while (!Serial) {
    ;
  }

  Serial.println("Serial ready. Initializing LED...");
  // initialize the LED pins as outputs, 
  beginLED();
  setoff();

  Serial.println("LED ready. Initializing PDM microphone...");
  // initialize the PDM microphoneand set the callback function to be called when data is available
  PDM.onReceive(onPDMdata);
  if (!PDM.begin(1, 16000)) {
    Serial.println("Failed to initialize PDM!");
    while (1) {
      delay(1000);
    }
  }

}

void loop() {
  int voice = 120;
  int noise = 700;

  if (samplesRead > 0) {
    long sumSquares = 0;

    for (int i = 0; i < samplesRead; i++) {
      long sample = sampleBuffer[i];
      sumSquares += sample * sample;
    }

    int rms = sqrt(sumSquares / samplesRead);
    Serial.println("RMS: " + String(rms));
    if (rms >= noise) {
      setred();
    } else if (rms >= voice) {
      setgreen();
    } else {
      setyellow();
    }
    samplesRead = 0; // Reset for the next batch of samples
  }
}
