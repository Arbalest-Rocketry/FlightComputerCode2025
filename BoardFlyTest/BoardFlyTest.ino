#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <Adafruit_BMP280.h>
#include <utility/imumaths.h>
#include <SPI.h>
#include <KalmanFilter.h>
#include <SD.h>
#include <rocket_stages.h>
#include <RH_RF95.h>

#include <Adafruit_BME280.h>


#define RFM95_CS 9
#define RFM95_RST 8
#define RFM95_INT 7
#define RF95_FREQ 915.9888

  

Adafruit_BME280 bme;
Adafruit_BNO055 bno = Adafruit_BNO055();
KalmanFilter filter;
const int chipSelect = 10;
detector stage_detector;
File dataFile;
double running_time;
double acceleration, altitude, filtered_acceleration, filtered_altitude, angleX, angleY;

RH_RF95 rf95(RFM95_CS, RFM95_INT);

int whiteLED = 20;
int redLED = 21;
int Buzzer = 22;

String filenameObj;



void setup() {
    
  // put your setup code here, to run once:
  Serial.begin(9600);
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

  pinMode(6,OUTPUT);
  Serial.println("CAMERA ON");
  




  pinMode(whiteLED, OUTPUT);
  pinMode(redLED, OUTPUT);
  pinMode(Buzzer, OUTPUT);
  if(!bno.begin())
  {
    Serial.println("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    while(1);
  }
  if(!bme.begin()){
    Serial.println("No bme detected");
    while(1);
  }
  if (!SD.begin(chipSelect)) {
    Serial.println("initialization failed. Things to check:");
    while (1);
  }
  //SD.remove("FLIGHTDATA.txt");//remove the previous data
  filenameObj = FileName(); 
  File dataFile = SD.open(filenameObj.c_str(), FILE_WRITE);

  imu::Vector<3> a = bno.getVector(Adafruit_BNO055::VECTOR_ACCELEROMETER);
  imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
  acceleration = a.z();
  altitude = bme.readAltitude(1024.1);//change depend on the pressure value
  filter.initial(altitude,acceleration);
  initAngle(euler.x(), euler.y());
  Serial.println(filenameObj);
  
}

void loop() {
  //Serial.println(filenameObj);
  running_time = millis();
  // put your main code here, to run repeatedly:
  digitalWrite(whiteLED, LOW);
  digitalWrite(redLED, LOW);
  
  imu::Vector<3> a = bno.getVector(Adafruit_BNO055::VECTOR_ACCELEROMETER);
  imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
  acceleration = a.z();
  altitude = bme.readAltitude(1024.1);//change depend on the pressure value
  angleX = euler.x();
  angleY = euler.y();
  filter.filter(altitude,acceleration);
  //Serial.println(a.z());
  //wait 10s for the filter to completely run through
  
  while(running_time >= 10000){
    
    imu::Vector<3> a = bno.getVector(Adafruit_BNO055::VECTOR_ACCELEROMETER);
    imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
    acceleration = a.z();
    altitude = bme.readAltitude(1024.1);//change depend on the pressure value
    angleX = euler.x();
    angleY = euler.y();
    filter.filter(altitude,acceleration);
    File dataFile = SD.open(filenameObj.c_str(), FILE_WRITE);
    //Serial.println(a.z());
    
    filtered_acceleration = filter.update_state[2];
    filtered_altitude = filter.update_state[0];
    //logData(dataFile,acceleration,altitude,filtered_acceleration,filtered_altitude,stage_detector.stage,running_time);
    //transmit_data(angleX, angleY, altitude, filtered_altitude, acceleration, filtered_acceleration, stage_detector.stage, rf95);
    //-------------------
    //Serial.print("acc: ");Serial.println(filtered_acceleration);
    //Serial.print("alt: ");Serial.println(filtered_altitude);
    //Serial.print("angleX: "); Serial.println(angleX);
    //Serial.print("angleY: "); Serial.println(angleY);
    //-------------------
    switch(stage_detector.stage){
        case 0:
            digitalWrite(whiteLED, HIGH);
            digitalWrite(redLED, HIGH);
            digitalWrite(6,HIGH);
            
            logData(dataFile,acceleration,altitude,filtered_acceleration,filtered_altitude,stage_detector.stage,running_time);
            transmit_data(angleX, angleY, altitude, filtered_altitude, acceleration, filtered_acceleration, stage_detector.stage, rf95);
            Serial.println("Prelaunch State");
            if(acceleration <= 0){//change with orientation, 0 because we have it opposite dir with gravity
                stage_detector.stage = 1;
            }
            break;
        case 1:
            /*
            if(tiltLock(euler.x(), euler.y())){
                cutoff();// safety check, check angle then put in all
                stage_detector.stage = 9;
            }*/
            
            logData(dataFile,acceleration,altitude,filtered_acceleration,filtered_altitude,stage_detector.stage,running_time);
            transmit_data(angleX, angleY, altitude, filtered_altitude, acceleration, filtered_acceleration, stage_detector.stage, rf95);
            Serial.println("Launch Detected, wating for 1st burnout");
            if(stage_detector.burnout(acceleration)){//change with orientation
                deploySeperation();// deploy pyro for seperation
                stage_detector.stage = 2;
            }
            break;
        case 2:
            /*
            if(tiltLock(euler.x(), euler.y())){
                cutoff();// safety check, check angle then put in all
                stage_detector.stage = 9;
            }*/
            logData(dataFile,acceleration,altitude,filtered_acceleration,filtered_altitude,stage_detector.stage,running_time);
            transmit_data(angleX, angleY, altitude, filtered_altitude, acceleration, filtered_acceleration, stage_detector.stage, rf95);
            Serial.println("burnout detected, waiting for apogee");
            if(stage_detector.apogee(altitude)){
                deployS2drouge();//deploy second stage drouge
                tone(Buzzer, 1000);
                delay(100);
                noTone(Buzzer);
                stage_detector.stage = 3;
                digitalWrite(whiteLED, LOW);
                digitalWrite(redLED, LOW);
            }
            break;
        case 3:
            /*
            if(tiltLock(euler.x(), euler.y())){
                cutoff();// safety check, check angle then put in all
                stage_detector.stage = 9;
            }*/
            logData(dataFile,acceleration,altitude,filtered_acceleration,filtered_altitude,stage_detector.stage,running_time);
            transmit_data(angleX, angleY, altitude, filtered_altitude, acceleration, filtered_acceleration, stage_detector.stage, rf95);
            Serial.println("Second stage Apogee detected, wait for main chute deploy");
            if(true){
                deployS2main();//deploy second stage main chute
                stage_detector.stage = 4;
            }
            break;
        case 4:
            /*
            if(tiltLock(euler.x(), euler.y())){
                cutoff();// safety check, check angle then put in all
                stage_detector.stage = 9;
            }*/
            logData(dataFile,acceleration,altitude,filtered_acceleration,filtered_altitude,stage_detector.stage,running_time);
            transmit_data(angleX, angleY, altitude, filtered_altitude, acceleration, filtered_acceleration, stage_detector.stage, rf95);
            Serial.println("Main Chute deployed, waitting for landing");
            if(stage_detector.land(altitude)){
                stage_detector.stage = 5;
            }
            break;
        case 5:
            /*
            if(tiltLock(euler.x(), euler.y())){
                cutoff();// safety check, check angle then put in all
                stage_detector.stage = 9;
            }*/
            logData(dataFile,acceleration,altitude,filtered_acceleration,filtered_altitude,stage_detector.stage,running_time);
            transmit_data(angleX, angleY, altitude, filtered_altitude, acceleration, filtered_acceleration, stage_detector.stage, rf95);
            Serial.println("Land detected, turnon low power mode");
            bno.setMode(OPERATION_MODE_CONFIG);
            bno.enterSuspendMode();
            bno.setMode(OPERATION_MODE_NDOF);
            bno.setExtCrystalUse(false);
            bme.setSampling(Adafruit_BME280::MODE_SLEEP);
            stage_detector.stage = 6;
            break;
        case 6:
            /*
            if(tiltLock(euler.x(), euler.y())){
                cutoff();// safety check, check angle then put in all
                stage_detector.stage = 9;
            }*/
            
            Serial.println("On low power mode");
            logData(dataFile,acceleration,altitude,filtered_acceleration,filtered_altitude,stage_detector.stage,running_time);
            transmit_data(angleX, angleY, altitude, filtered_altitude, acceleration, filtered_acceleration, stage_detector.stage, rf95);
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
        case 9:
            Serial.println("Angle go over safety, the goose is cooked! ");
            transmit_data(angleX, angleY, altitude, filtered_altitude, acceleration, filtered_acceleration, stage_detector.stage, rf95);
            digitalWrite(whiteLED, LOW);
            digitalWrite(redLED, LOW);
            cutoff();

    }
    //PRINT DATA---
    Serial.print("Altitude: ");Serial.print(filtered_altitude);Serial.print(", ");
    Serial.print("Acceleration: ");Serial.print(filtered_acceleration);Serial.print(", ");
    Serial.print("angleX: ");Serial.print(angleX);Serial.print(", ");
    Serial.print("angleY: ");Serial.print(angleY);Serial.print(", ");
    Serial.print("Stage: "); Serial.println(stage_detector.stage);
  //-------------
  }


}
