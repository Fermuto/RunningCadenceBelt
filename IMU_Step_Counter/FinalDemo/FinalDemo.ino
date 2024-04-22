#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include <Filters.h>
#include <Filters/Butterworth.hpp>

uint16_t BNO055_SAMPLERATE_DELAY_MS = 10;
Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28, &Wire);
#define num_sam 10
const double f_s = 100;
const double f_c = 25;
const double f_n = 2 * f_c / f_s;

int steps = 0;
int cadence;
int target = 180;

float threshold = 0;  // TODO: Find this number (Testing/Debugging)
int isStep = 0;
int afterStep = 1;
float accelx[num_sam];
float accely[num_sam];
float ax[num_sam];
float ay[num_sam];
float az[num_sam];

// Build the Low-pass Filter (Outlined for 2nd Order)
float b[] = {0.007777, 0.01555399, 0.007777};       // Filter coeffs. can be obtained using the
float a[] = {-1.73550016, 0.76660814};              // Python code provided in the directory

// Holds previous values for filter use
float x[3];
float y[3];

void setup() {
  // Open Serial
  Serial.begin(115200);
  while (!Serial) delay(10);
  Serial.println("Sensor Demonstration"); Serial.println("");

  // In case can't communicate with BNO055
  if (!bno.begin())
  {
    Serial.print("BNO055 Not Detected!");
    while (1);
  }

  delay(1000);
}

void loop() {
  // grab Linear Acceleration Data
  sensors_event_t LinearAccelerationData;
  bno.getEvent(&LinearAccelerationData, Adafruit_BNO055::VECTOR_LINEARACCEL);
  double x = -1000000, y = -1000000 , z = -1000000;
  Serial.print("Linear:");
  x = LinearAccelerationData.acceleration.x;
  y = LinearAccelerationData.acceleration.y;
  z = LinearAccelerationData.acceleration.z;
  

  for (int i = 0; i < num_sam; i++){
    bno.getEvent(&LinearAccelerationData, Adafruit_BNO055::VECTOR_LINEARACCEL);
    ax[i] = LinearAccelerationData.acceleration.x;
    ay[i] = LinearAccelerationData.acceleration.y;
    az[i] = LinearAccelerationData.acceleration.z;
    Serial.print("\tx= ");
    Serial.print(ax[i]);
    Serial.print(" |\ty= ");
    Serial.print(ay[i]);
    Serial.print(" |\tz= ");
    Serial.println(az[i]);
    accelx[i] = sqrt(ax[i]*ax[i]);
    delay(10);

  //--------------------------------Apply the Filter--------------------------------
  for (int i = 0; i < num_sam; i++){
    x[0] = accelx[i];
    y[0] = b[0]*x[0] + b[1]*x[1] + b[2]*x[2] - a[0]*y[1] - a[1]*y[2];
    x[2] = x[1];
    x[1] = x[0];
    y[2] = y[1];
    y[1] = y[0];

    accelx[i] = y[0];
  //--------------------------------------------------------------------------------
  
    if ((accel[i] > threshold) && (isStep == 0)){
      steps = steps + 1;
      isStep = 1;
    }
    else if ((accel[i] < threshold) && (isStep == 1)){
      isStep = 0;
    }
  }


  
}
