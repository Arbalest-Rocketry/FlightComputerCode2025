#include <Adafruit_Sensor.h>
#include <Wire.h>
#include <Adafruit_BMP280.h>
#include <KalmanFilter.h>
#include "rocket_stages.h"
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>

Adafruit_BMP280 bmp;
Adafruit_BNO055 bno = Adafruit_BNO055();

KalmanFilter filter;
detector apogee_detector;

double raw_altitude;

double test_alt = 0;
double test_acc = 0;
int test_count = 0;
void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  if(!bno.begin())
  {
    Serial.println("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    while(1);
  }
  if(!bmp.begin()){
    Serial.println("No bmp detected");
    while(1);
  }
  /*
  imu::Vector<3> accel = bno.getVector(Adafruit_BNO055::VECTOR_ACCELEROMETER);
  raw_acceleration = accel.z();
  raw_altitude = bmp.readAltitude(1050.35);
  filter.initial(raw_altitude,raw_acceleration);
  */
  while(apogee_detector.stage != 8){
    
    if(test_count<=5){
      test_acc += 2;
      test_alt += 100;
      test_count++;
      state_machine(apogee_detector, test_acc,test_alt);
    }
    if((test_count <=10) & (test_count > 5)){
      test_acc -= 3;
      test_alt += 100;
      test_count++;
      state_machine(apogee_detector, test_acc,test_alt);
    }
    if((test_count <=15) & (test_count > 10)){
      test_acc += 2;
      test_alt += 100;
      test_count++;
      state_machine(apogee_detector, test_acc,test_alt);
    }
    if((test_count <=20) & (test_count > 15)){
      test_acc -= 3;
      test_alt += 100;
      test_count++;
      state_machine(apogee_detector, test_acc,test_alt);
    }
    if((test_count <=30) & (test_count > 20)){
      test_alt -= 100;
      test_count++;
      state_machine(apogee_detector, test_acc,test_alt);
    }
    if(test_count >30){
      if(test_alt != 0){
        test_alt -= 100;
        state_machine(apogee_detector, test_acc,test_alt);
      }
      else{state_machine(apogee_detector, test_acc,test_alt);}
    }
    Serial.print(test_count);Serial.print(" "); Serial.print(test_acc);Serial.print(" "); Serial.println(test_alt);
    delay(500);
  }
  

}

void loop() {
  // put your main code here, to run repeatedly:

}
