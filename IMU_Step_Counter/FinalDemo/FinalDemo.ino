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

int steps = 0;
int cadence;
int target = 180;
int button = 0;
int lastbutton = 0;
int buttonpressed = 0;
int buttonused = 0;

float threshold = 17;
int isStep = 0;
unsigned long lastsig = 0;
unsigned long lastcad = 0;
unsigned long buttonheld = 0;
unsigned long now = 0;
unsigned long samplestart = 0;
unsigned long sampleend = 0;
unsigned long samplediff = 0;
float accelx[num_sam];
float ax[num_sam];
float ay[num_sam];
float az[num_sam];

//// Build the Low-pass Filter (Outlined for 2nd Order)
//float b[] = {0.007777, 0.01555399, 0.007777};       // Filter coeffs. can be obtained using the
//float a[] = {-1.73550016, 0.76660814};              // Python code provided in the directory
//
//// Holds previous values for filter use
//float x[3];
//float y[3];

void setup() {
  // Open Serial
  Serial.begin(115200);
  while (!Serial) delay(10);
  Serial.println("Sensor Demonstration"); Serial.println("");

  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(4, OUTPUT);
  pinMode(5, INPUT);
  digitalWrite(LED_BUILTIN, LOW);
  digitalWrite(4, LOW);
  

  // In case can't communicate with BNO055
  if (!bno.begin())
  {
    Serial.print("BNO055 Not Detected!");
    while (1);
  }

  delay(1000);
}

void loop() {
  button = digitalRead(5);
  samplestart = millis();
  // grab Linear Acceleration Data
  sensors_event_t LinearAccelerationData;
  bno.getEvent(&LinearAccelerationData, Adafruit_BNO055::VECTOR_LINEARACCEL);

  for (int i = 0; i < num_sam; i++){
    samplestart = millis();
    bno.getEvent(&LinearAccelerationData, Adafruit_BNO055::VECTOR_LINEARACCEL);
    ax[i] = LinearAccelerationData.acceleration.x;
    ay[i] = LinearAccelerationData.acceleration.y;
    az[i] = LinearAccelerationData.acceleration.z;
    if (i == 9){ // Really only for human viewing, don't need to see a 100Hz output.
      Serial.print("\tx= ");
      Serial.print(ax[i]);
      Serial.print(" |\ty= ");
      Serial.print(ay[i]);
      Serial.print(" |\tz= ");
      Serial.print(az[i]);
    }
    accelx[i] = sqrt(ax[i]*ax[i]);

    // Attempt to maintain a 100Hz sampling rate
    sampleend = millis();
    samplediff = samplestart - sampleend;
    if (samplediff >= 10){
      samplediff = 10;
    }
    delay(10 - samplediff);
  
  //--------------------------------Apply the Filter--------------------------------
  
  //--------------------------------------------------------------------------------
  
    if ((accelx[i] > threshold) && (isStep == 0)){
      steps = steps + 1;
      isStep = 1;
    }
    else if ((accelx[i] < threshold) && (isStep == 1)){
      isStep = 0;
    }
  }

  now = millis();

  // Update cadence ever 0.2s
  if ((now - lastcad) > 200){
    cadence = (steps*60000)/(now-lastsig);
  }

  if ((button == 1) && (lastbutton == 0)){
    buttonheld = millis();
  }
  else if ((button == 0) && (lastbutton == 1)){
    if (now - buttonheld > 2000){
      target = 180;
    }
    else{
      target = target - 5;
    }
    buttonused = 0;
  }


  Serial.print(" |\tSteps= ");
  Serial.print(steps);
  Serial.print(" |\tCadence= ");
  Serial.print(cadence);
  Serial.print(" |\tTarget= ");
  Serial.print(target);
  Serial.print(" |\tButton= ");
  Serial.println(button);

  
  
  // Evaluate cadence every 10 seconds (effectively, windowing)
  if ((now - lastsig) > 10000){
    if (cadence > (target+10)){
      digitalWrite(4, HIGH);
      digitalWrite(LED_BUILTIN, HIGH);
      delay(500);
      digitalWrite(4, LOW);
      digitalWrite(LED_BUILTIN, LOW);
    }
    else if (cadence < (target-10)){
      if (cadence > 120){
        digitalWrite(4, HIGH);
        delay(300);
        digitalWrite(4, LOW);
        delay(100);
        digitalWrite(4, HIGH);
        delay(300);
        digitalWrite(4, LOW);

      }
    }
    lastsig = now;
    steps = 0; // Reset step count evry 10 seconds to acquire new window
  }

  
  lastbutton = button;
  
}
