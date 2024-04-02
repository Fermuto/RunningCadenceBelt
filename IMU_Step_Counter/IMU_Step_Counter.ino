#include <Arduino.h>
#include <Adafruit_BNO08x.h>
#include <Time.h>

// SPI mode:       THIS MIGHT BE RIGHT?? IDK
#define BNO08X_CS 10
#define BNO08X_INT 9
#define BNO08X_RESET 5

#define num_sam 150

Adafruit_BNO08x bno08x(BNO08X_RESET);
sh2_SensorValue_t sensorValue;

void setup() {
  Serial.begin(115200);   // TODO: It might also be 921600
  
  if (!bno08x.begin_SPI(BNO08X_CS, BNO08X_INT)) {
    Serial.println("Failed to find BNO08x chip");
  }
  Serial.println("BNO08x Found!");
}

unsigned long last;
unsigned long now;
unsigned long t = 0;
unsigned long lastsig = 0;

int steps = 0;

float threshold = ;  // TODO: Find this number (Testing/Debugging)
int isStep = 0;
float accel[num_sam];
float ax[num_sam];
float ay[num_sam];
float az[num_sam];

// Build the Low-pass Filter (Outlined for 2nd Order)
float b[] = { , , };      // Filter coeffs. can be obtained using the
float a[] = { , };        // Python code I provided in the directory

// Holds previous values for filter use
float x[3];
float y[3];

void loop() { 
  last = millis();

  for (int i = 0; i < num_sam; i++){
    ax[i] = sensorValue.un.accelerometer.x    // TODO: Replace these with
    ay[i] = sensorValue.un.accelerometer.y    // the correct pin numbers
    az[i] = sensorValue.un.accelerometer.z
    accel[i] = sqrt( (ax[i]*ax[i]) + (ay[i]*ay[i]) + (az[i]*az[i]) )
  }

//--------------------------------Apply the Filter--------------------------------
  for (int i = 0; i < num_sam; i++){
    x[0] = accel[i];
    y[0] = b[0]*x[0] + b[1]*x[1] + b[2]*x[2] - a[1]*y[1] - a[2]*y[2];

    x[2] = x[1];
    x[1] = x[0];
    y[2] = y[1];
    y[1] = y[0];

    accel[i] = y[0]
//--------------------------------------------------------------------------------
  
    delay(50);        // TODO: Will adjust this during debugging

    if ((accel[i] > threshold) && (isStep == 0)){
      steps = steps + 1;
      isStep = 1;
    }
    if ((accel[i] < threshold) && (isStep == 1)){
      isStep = 0;
    }
  }

  // TODO: Fix Crude Algorithm, make it work
  // TODO: Calculate SPM, Send DS to Haptic Board for Feedback
  now = millis();
  t = t + (now-last);
  cadence = (steps*60000)/(t)
  
  if ((millis() - lastsig) > 10000){
    if ((cadence > 185) || (cadence < 175)){
       digitalWrite(pin, HIGH)
    }
    lastsig = millis();
  }
  
}
