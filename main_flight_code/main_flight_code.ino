#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <Adafruit_BMP280.h>
#include <utility/imumaths.h>
#include <SPI.h>
#include <KalmanFilter.h>
#include <SD.h>
#include <rocket_stages.h>

Adafruit_BMP280 bmp;
Adafruit_BNO055 bno = Adafruit_BNO055();
KalmanFilter filter;
const int chipSelect = 10;
detector stage_detector;
File dataFile;
double time;
double acceleration, altitude, filtered_acceleration, filtered_altitude;




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
  if (!SD.begin(chipSelect)) {
    Serial.println("initialization failed. Things to check:");
    while (1);
  }
  SD.remove("FLIGHTDATA.txt");//remove the previous data

  imu::Vector<3> a = bno.getVector(Adafruit_BNO055::VECTOR_ACCELEROMETER);
  acceleration = a.z();
  altitude = bmp.readAltitude(1012.9);//change depend on the pressure value
  filter.initial(altitude,acceleration);
  time = millis();
}

void loop() {
  // put your main code here, to run repeatedly:
  
  imu::Vector<3> a = bno.getVector(Adafruit_BNO055::VECTOR_ACCELEROMETER);
  acceleration = a.z();
  altitude = bmp.readAltitude(1012.9);//change depend on the pressure value
  filter.filter(altitude,acceleration);
  //wait 10s for the filter to completely run through
  while(time >= 10000){
    filtered_acceleration = filter.current_state[2];
    filtered_altitude = filter.current_state[0];
    logData(dataFile,acceleration,altitude,filtered_acceleration,filtered_altitude,stage_detector.stage,time);
    transmitData();
    switch(stage_detector.stage){
        case 0:
            logData(dataFile,acceleration,altitude,filtered_acceleration,filtered_altitude,stage_detector.stage,time);
            transmitData();
            Serial.println("Prelaunch State");
            if(acceleration >= 8){//change with orientation
                stage_detector.stage = 1;
            }
            break;
        case 1:
            logData(dataFile,acceleration,altitude,filtered_acceleration,filtered_altitude,stage_detector.stage,time);
            transmitData();
            Serial.println("Launch Detected, wating for 1st burnout");
            if(stage_detector.burnout(acceleration)){//change with orientation
                stage_detector.stage = 2;
            }
            break;
        case 2:
            logData(dataFile,acceleration,altitude,filtered_acceleration,filtered_altitude,stage_detector.stage,time);
            transmitData();
            Serial.println("Burnout detected, waiting for seperation");
            //pyro turn on to separate the stage
            deployPyro();//need to add pin number here
            delay(6000);//wait 6 seconds then change to ignite stage
            if(acceleration>=5){
                stage_detector.stage = 3;
            }
            break;
        case 3:
            logData(dataFile,acceleration,altitude,filtered_acceleration,filtered_altitude,stage_detector.stage,time);
            transmitData();
            Serial.println("Igniting, wait for second burnout");
            if(stage_detector.burnout(acceleration)){
                stage_detector.stage = 4;
            }
            break;
        case 4:
            logData(dataFile,acceleration,altitude,filtered_acceleration,filtered_altitude,stage_detector.stage,time);
            transmitData();
            Serial.println("Second burnout detected, waiting for apogee");
            if(stage_detector.apogee(altitude)){
                deployPyro();//need to add pin number here
                stage_detector.stage = 5;
            }
            break;
        case 5:
            logData(dataFile,acceleration,altitude,filtered_acceleration,filtered_altitude,stage_detector.stage,time);
            transmitData();
            Serial.println("Apogee detected, wait for main chute deploy");
            if(altitude <= 457.2){
                stage_detector.stage = 6;
            }
            break;
        case 6:
            logData(dataFile,acceleration,altitude,filtered_acceleration,filtered_altitude,stage_detector.stage,time);
            transmitData();
            Serial.println("Main Chute deployed, waitting for landing");
            if(stage_detector.land(altitude)){
                stage_detector.stage = 7;
            }
            break;
        case 7:
            logData(dataFile,acceleration,altitude,filtered_acceleration,filtered_altitude,stage_detector.stage,time);
            transmitData();
            Serial.println("Land detected, turnon low power mode");
            bno.setMode(OPERATION_MODE_CONFIG);
            bno.enterSuspendMode();
            bno.setMode(OPERATION_MODE_NDOF);
            bno.setExtCrystalUse(false);
            bmp.setSampling(Adafruit_BMP280::MODE_SLEEP);
            stage_detector.stage = 8;
            break;
        case 8:
            Serial.println("On low power mode");
            transmitData();
            break;
    }
  }


}
