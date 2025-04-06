#include <Adafruit_Sensor.h>
#include <Wire.h>
#include <Adafruit_BMP085.h>
#include <KalmanFilter.h>
#include "NewApogee.h"

Adafruit_BMP085 bmp;
KalmanFilter filter;
detector apogee_detector;

double raw_altitude;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  if (!bmp.begin()) {
    Serial.println("Could not find a valid BMP180 sensor, check wiring!");
    while (1)
      ;  // Halt if sensor not found
  }
  Serial.println("BMP180 Found!");
  raw_altitude = bmp.readAltitude(1013.25);
  

  while(apogee_detector.reached == false){
    Serial.println(raw_altitude);
    //apogee_detector.apogee(raw_altitude);
    //delay(1000);
  }

}

void loop() {
  // put your main code here, to run repeatedly:

}
