#include <Arduino.h>
#include <Adafruit_BNO08x.h>
#include <Time.h>

// SPI mode:
#define ESP32_MOSI GPIO_NUM_13
#define ESP32_MISO GPIO_NUM_12
#define ESP32_SCLK GPIO_NUM_14
#define ESP32_CS GPIO_NUM_15

#define BNO08X_CS GPIO_NUM_10
#define BNO08X_INT GPIO_NUM_9
#define BNO08X_RESET GPIO_NUM_5

// #define Test5 GPIO_NUM_5

#define num_sam 150

Adafruit_BNO08x bno08x(BNO08X_RESET);
sh2_SensorValue_t sensorValue;

unsigned long now;
unsigned long lastsig = 0;

int steps = 0;
int cadence;
int target = 180;

float threshold = 0;  // TODO: Find this number (Testing/Debugging)
int isStep = 0;
float accel[num_sam];
float ax[num_sam];
float ay[num_sam];
float az[num_sam];

// Build the Low-pass Filter (Outlined for 2nd Order)
float b[] = {0.007777, 0.01555399, 0.007777};       // Filter coeffs. can be obtained using the
float a[] = {-1.73550016, 0.76660814};              // Python code I provided in the directory

// Holds previous values for filter use
float x[3];
float y[3];

void setup() {
  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(GPIO_NUM_4, OUTPUT);
  Serial.begin(115200);
  while (!Serial) delay(10);

  Serial.println("ax,ay,az,Net_Acceleration");
  bno08x.begin_SPI(BNO08X_CS, BNO08X_INT);
  if (bno08x.begin_SPI(BNO08X_CS, BNO08X_INT)) {
    Serial.println("Failed to find BNO08x chip");
    digitalWrite(LED_BUILTIN, HIGH);
  }
  else {
    digitalWrite(LED_BUILTIN, LOW);
  }
  Serial.println("BNO08x Found!");
  if (bno08x.wasReset()) {
    //Serial.println("Failed to find BNO08x chip");
    digitalWrite(GPIO_NUM_4, HIGH);
  }
  else {
    digitalWrite(GPIO_NUM_4, LOW);
  }
}

void loop() { 
  // Serial.print('*');
  //digitalWrite(LED_BUILTIN, HIGH);

  for (int i = 0; i < num_sam; i++){
    ax[i] = sensorValue.un.accelerometer.x;    // TODO: Fix this
    Serial.print(ax[i]);
    Serial.print("\t");
    delay(1);
    ay[i] = sensorValue.un.accelerometer.y;
    Serial.print(ay[i]);
    Serial.print("\t");
    delay(1);
    az[i] = sensorValue.un.accelerometer.z;
    Serial.print(az[i]);
    Serial.print("\t");
    delay(1);
    accel[i] = sqrt( (ax[i]*ax[i]) + (ay[i]*ay[i]) + (az[i]*az[i]) );
    Serial.println(accel[i]);
    delay(1);
  }

  //digitalWrite(LED_BUILTIN, LOW);
  //--------------------------------Apply the Filter--------------------------------
  for (int i = 0; i < num_sam; i++){
    x[0] = accel[i];
    y[0] = b[0]*x[0] + b[1]*x[1] + b[2]*x[2] - a[1]*y[1] - a[2]*y[2];

    x[2] = x[1];
    x[1] = x[0];
    y[2] = y[1];
    y[1] = y[0];

    accel[i] = y[0];
  //--------------------------------------------------------------------------------
  
    delay(500);        // TODO: Will adjust this during debugging

    if ((accel[i] > threshold) && (isStep == 0)){
      steps = steps + 1;
      isStep = 1;
    }
    if ((accel[i] < threshold) && (isStep == 1)){
      isStep = 0;
    }
  }

  // TODO: Fix Crude Algorithm, make it work

  now = millis();
  cadence = (steps*60000)/(now);
  //Serial.println(cadence);
  
  if ((now - lastsig) > 10000){
    if (cadence > (target+5)){
        // digitalWrite(pin, HIGH)
        delay(1000);
        // digitalWrite(pin, LOW)
    }
    if (cadence < (target-5)){
        // digitalWrite(pin, HIGH)
        delay(500);
        // digitalWrite(pin, LOW)
        delay(250);
        // digitalWrite(pin, HIGH)
        delay(500);
        // digitalWrite(pin, LOW)
    }
    lastsig = now;
  }
  
}
