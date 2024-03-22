#include <Arduino.h>
#include <Adafruit_BNO08x.h>

// SPI mode:       I HAVE NO IDEA IF THIS IS CORRECT
#define BNO08X_CS 10        // TODO: Diagnose this
#define BNO08X_INT 9
#define BNO08X_RESET -1

#define num_sam 150

void setup() {
  Serial.begin(921600);   // TODO: It might also be 115200

  // TODO: Complete Setup
  
}

int steps = 0;

float threshold = ;  // TODO: Find this number (Testing/Debugging)
int isStep = 0;
float accel[num_sam];
float ax[num_sam];
float ay[num_sam];
float az[num_sam];

// Build the Low-pass Filter (Outlined for 2nd Order)
float b[] = { , , };  // Filter coeffs. can be obtained using the
float a[] = { , };        // Python code I provided in the directory

// Holds previous values for filter use
float x[3];
float y[3];

void loop() { 
  // Lets pretend all the setup stuff is taken care of.
  for (int i = 0; i < num_sam; i++){
    ax[i] = float(analogRead(pin))    // TODO: Replace these with
    ay[i] = float(analogRead(pin))    // the correct pin numbers
    az[i] = float(analogRead(pin))
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
  
}
