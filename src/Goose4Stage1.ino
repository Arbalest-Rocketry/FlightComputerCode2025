#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include <SPI.h>
#include <KalmanFilter.h>
#include <SD.h>
#include <rocket_stages.h>
#include <RH_RF95.h>
#include <Adafruit_BME280.h>
//Goose 4 Stage 1 Flight Computer Software by Tom Pham
//Pins-------------
#define RFM95_CS 10
#define RFM95_RST 6
#define RFM95_INT 7
#define RF95_FREQ 915.9888
int greenLED = 3;
int redLED = 4;
int blueLED = 2;
int Buzzer = 5;
const int chipSelect = 9;
const int Camera = 6;
//Objects----------
Adafruit_BME280 bme;
Adafruit_BNO055 bno = Adafruit_BNO055();
KalmanFilter filter;
detector stage_detector;
File dataFile;
RH_RF95 rf95(RFM95_CS, RFM95_INT);
String filenameObj;
//Variable---------
double running_time;
double acceleration, altitude, filtered_acceleration, filtered_altitude, angleX, angleY, angleZ;
//-----------------




void setup() {
  Serial.begin(9600);
  //Led, Buzzer, Camera----------------------------------
  pinMode(blueLED, OUTPUT);
  pinMode(redLED, OUTPUT);
  pinMode(greenLED, OUTPUT);
  pinMode(Buzzer, OUTPUT);
  pinMode(Camera, OUTPUT);
  //-----------------------------------------------------
  //Radio Frequency Set Up-------------------------------
  pinMode(RFM95_RST, OUTPUT);
  digitalWrite(RFM95_RST, HIGH);
  digitalWrite(RFM95_RST, LOW);
  delay(10);
  digitalWrite(RFM95_RST, HIGH);
  delay(10);
  if (!rf95.init()) {
    Serial.println("❌ LoRa radio init failed!");
    while (1);
  }
  Serial.println("✅ LoRa radio initialized!");
  if (!rf95.setFrequency(RF95_FREQ)) {
    Serial.println("❌ LoRa set frequency failed!");
    while (1);
  }
  Serial.print("📡 Set LoRa Frequency to: "); Serial.println(RF95_FREQ);    
  rf95.setTxPower(20, false);
  //IMU--------------------------------------------------
  if(!bno.begin())
    Serial.println("❌ BNO055 IMU init failed!");
    while(1);
  }
  Serial.println("✅ BNO055 IMU initialized!");
  imu::Vector<3> a = bno.getVector(Adafruit_BNO055::VECTOR_ACCELEROMETER);
  imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
  acceleration = a.z();
  initAngle(euler.x(), euler.y());
  //Barometer---------------------------------------------
  if(!bme.begin()){
    Serial.println("❌ BME280 Barometer init failed!");
    while(1);
  }
  Serial.println("✅ BME280 Barometer initialized!");
  altitude = bme.readAltitude(1024.1); //change pressure depend on where u at
  //SD----------------------------------------------------
  if (!SD.begin(chipSelect)) {
    Serial.println("❌ SD init failed!");
    while (1);
  }
  Serial.println("✅ SD initialized!");
  filenameObj = FileName(); 
  File dataFile = SD.open(filenameObj.c_str(), FILE_WRITE);
  Serial.println(filenameObj);
  //-----------------------------------------------------
  filter.initial(altitude,acceleration);//initialize kalman filter
}

void loop() {
  running_time = millis();
  digitalWrite(greenLED, LOW);
  digitalWrite(redLED, LOW);
  digitalWrite(blueLED, LOW);
  digitalWrite(Camera, HIGH);
  imu::Vector<3> a = bno.getVector(Adafruit_BNO055::VECTOR_ACCELEROMETER);
  imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
  acceleration = a.z();
  altitude = bme.readAltitude(1024.1);//change depend on the pressure value
  angleX = euler.x();
  angleY = euler.y();
  angleZ = euler.z();
  filter.filter(altitude,acceleration);
  while(running_time >= 10000){
    imu::Vector<3> a = bno.getVector(Adafruit_BNO055::VECTOR_ACCELEROMETER);
    imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
    acceleration = a.z();
    altitude = bme.readAltitude(1024.1);
    angleX = euler.x();
    angleY = euler.y();
    angleZ = euler.z();
    filter.filter(altitude,acceleration);
    File dataFile = SD.open(filenameObj.c_str(), FILE_WRITE);
    filtered_acceleration = filter.update_state[2];
    filtered_altitude = filter.update_state[0];
    //BEGIN OF STATE MACHINE---------------------------------
    switch(stage_detector.stage){
      case 0:
        digitalWrite(greenLED, HIGH);
        digitalWrite(redLED, HIGH);
        digitalWrite(blueLED, HIGH);
        logData(dataFile,acceleration,altitude,filtered_acceleration,filtered_altitude,stage_detector.stage,running_time);
        transmit_data(angleX, angleY, angleZ, altitude, filtered_altitude, acceleration, filtered_acceleration, stage_detector.stage, rf95);
        Serial.println("Prelaunch State");
        if(acceleration <= 0){//change with orientation, 0 because we have it opposite dir with gravity
          tage_detector.stage = 1;
        }
        break;
      case 1:
        //Safety Check----------------------
        if(tiltLock(euler.x(), euler.y())){
          cutoff();
          stage_detector.stage = 7;
        }//---------------------------------
        logData(dataFile,acceleration,altitude,filtered_acceleration,filtered_altitude,stage_detector.stage,running_time);
        transmit_data(angleX, angleY, angleZ, altitude, filtered_altitude, acceleration, filtered_acceleration, stage_detector.stage, rf95);
        Serial.println("Launch Detected, wating for 1st burnout");
        if(stage_detector.burnout(acceleration)){//change with orientation
          deploySeperation();// deploy pyro for seperation
          stage_detector.stage = 2;
        }
        break;
      case 2:
        //Safety Check----------------------
        if(tiltLock(euler.x(), euler.y())){
          cutoff();
          stage_detector.stage = 7;
        }//---------------------------------
        logData(dataFile,acceleration,altitude,filtered_acceleration,filtered_altitude,stage_detector.stage,running_time);
        transmit_data(angleX, angleY, angleZ, altitude, filtered_altitude, acceleration, filtered_acceleration, stage_detector.stage, rf95);
        Serial.println("burnout detected, waiting for apogee");
        if(stage_detector.apogee(altitude)){
          deployS1drouge();//deploy first stage drouge
          stage_detector.stage = 3;
          digitalWrite(greenLED, LOW);
          digitalWrite(redLED, LOW);
          digitalWrite(blueLED, LOW);
        }
        break;
        case 3:
          //Safety Check----------------------
          if(tiltLock(euler.x(), euler.y())){
            cutoff();
            stage_detector.stage = 7;
          }//---------------------------------
          logData(dataFile,acceleration,altitude,filtered_acceleration,filtered_altitude,stage_detector.stage,running_time);
          transmit_data(angleX, angleY, angleZ, altitude, filtered_altitude, acceleration, filtered_acceleration, stage_detector.stage, rf95);
          Serial.println("Apogee detected, wait for main chute deploy");
          if(true){//change to whatever height 80% apogee I think
            deployS1main();//deploy second stage main chute
            stage_detector.stage = 4;
          }
          break;
        case 4:
          //Safety Check----------------------
          if(tiltLock(euler.x(), euler.y())){
            cutoff();
            stage_detector.stage = 7;
          }//---------------------------------
          logData(dataFile,acceleration,altitude,filtered_acceleration,filtered_altitude,stage_detector.stage,running_time);
          transmit_data(angleX, angleY, angleZ, altitude, filtered_altitude, acceleration, filtered_acceleration, stage_detector.stage, rf95);
          Serial.println("Main Chute deployed, waiting for landing");
          if(stage_detector.land(altitude)){
            stage_detector.stage = 5;
          }
          break;
        case 5:
          //Safety Check----------------------
          if(tiltLock(euler.x(), euler.y())){
            cutoff();
            stage_detector.stage = 7;
          }//---------------------------------
          logData(dataFile,acceleration,altitude,filtered_acceleration,filtered_altitude,stage_detector.stage,running_time);
          transmit_data(angleX, angleY, angleZ, altitude, filtered_altitude, acceleration, filtered_acceleration, stage_detector.stage, rf95);
          Serial.println("Land detected, low power mode");
          bno.setMode(OPERATION_MODE_CONFIG);
          bno.enterSuspendMode();
          bno.setMode(OPERATION_MODE_NDOF);
          bno.setExtCrystalUse(false);
          bme.setSampling(Adafruit_BME280::MODE_SLEEP);
          stage_detector.stage = 6;
          break;
        case 6:
          transmit_data(angleX, angleY, angleZ, altitude, filtered_altitude, acceleration, filtered_acceleration, stage_detector.stage, rf95);
          Serial.println("On low power mode");
          tone(Buzzer, 2000);
          delay(1000);
          noTone(Buzzer);
          tone(Buzzer, 2000);
          delay(1000);
          noTone(Buzzer);
          tone(Buzzer, 2000);
          delay(1000);
          noTone(Buzzer);
          break;
      case 7:
          Serial.println("Angle go over safety, the goose is cooked! ");
          transmit_data(angleX, angleY, angleZ, altitude, filtered_altitude, acceleration, filtered_acceleration, stage_detector.stage, rf95);
          digitalWrite(greenLED, LOW);
          digitalWrite(redLED, LOW);
          digitalWrite(blueLED, LOW);
          cutoff();
          break;
    }
    /*
    data for debugging
    Serial.print("Altitude: ");Serial.print(filtered_altitude);Serial.print(", ");
    Serial.print("Acceleration: ");Serial.print(filtered_acceleration);Serial.print(", ");
    Serial.print("angleX: ");Serial.print(angleX);Serial.print(", ");
    Serial.print("angleY: ");Serial.print(angleY);Serial.print(", ");
    Serial.print("angleZ: ");Serial.print(angleZ);Serial.print(", ");
    Serial.print("Stage: "); Serial.println(stage_detector.stage);
    */
    //END OF STATE MACHINE-----------------------------------


  }

}
