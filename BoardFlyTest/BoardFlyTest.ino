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
#include <TinyGPSPlus.h>
#include <SoftwareSerial.h> 
#include <math.h>
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
const int Camera = 8;
//Variable---------
double running_time;
unsigned long delayStart = 0;
double acceleration, altitude, filtered_acceleration, filtered_altitude, angleX, angleY, angleZ, qw, qx, qy, qz, lat, lon, rx, temp, pressure, speed;
int sat, satcheck = 0;
char message[300] = "";
static const int RXPin = 0, TXPin = 1;
static const uint32_t GPSBaud = 9600;
unsigned long lastSDWrite = 0;
const unsigned long SDWriteInterval = 1000; // Write every 1s 
//Objects----------
Adafruit_BME280 bme;
Adafruit_BNO055 bno = Adafruit_BNO055();
KalmanFilter filter;
detector stage_detector;
File dataFile;
RH_RF95 rf95(RFM95_CS, RFM95_INT);
String filenameObj;
TinyGPSPlus gps;
SoftwareSerial ss(RXPin, TXPin);
//-----------------




void setup() {
  Serial.begin(115200);
  ss.begin(GPSBaud);
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
  if(!bno.begin()){
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
  //Serial.println(filenameObj);
  //-----------------------------------------------------
  filter.initial(altitude,acceleration);//initialize kalman filter
}
void loop() {
  running_time = millis();
  digitalWrite(greenLED, LOW);
  digitalWrite(redLED, LOW);
  digitalWrite(blueLED, LOW);
  
  while(ss.available()){
    gps.encode(ss.read());
  }
  imu::Vector<3> a = bno.getVector(Adafruit_BNO055::VECTOR_ACCELEROMETER);
  imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
  imu::Quaternion quat=bno.getQuat();
  acceleration = a.z();
  altitude = bme.readAltitude(1024.1);//change depend on the pressure value
  filter.filter(altitude,acceleration);
  sat = gps.satellites.value();
  lat = gps.location.lat();
  lon = gps.location.lng();
  rx = gps.charsProcessed();
  Serial.print(rx);Serial.print(", ");Serial.print(sat);Serial.print(", ");Serial.println(gps.failedChecksum());
  if(sat >= 4){
    digitalWrite(Camera, HIGH);
    satcheck = 1;
  }
  while(satcheck == 1){
    running_time = millis();
    while(ss.available()){
      gps.encode(ss.read());
    }
    imu::Vector<3> a = bno.getVector(Adafruit_BNO055::VECTOR_ACCELEROMETER);
    imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
    imu::Quaternion quat=bno.getQuat();
    acceleration = a.z();
    altitude = bme.readAltitude(1024.1);//change depend on the pressure value
    qw = quat.w();
    qx = quat.x();
    qy = quat.y();
    qz = quat.z();
    angleX = (atan2(2*((qw*qx) + (qy * qz)), 1 - 2*((qx*qx)+(qy*qy)))) * 180/PI;//yaw
    angleY = (-PI/2 + 2 * atan2(sqrt(1 + 2 * ((qw*qy)-(qz*qx))), sqrt(1 - 2 * ((qw*qy)-(qz*qx))))) * 180/PI;  //pitch
    angleZ = (atan2(2 * ((qw*qz)+(qx*qy)), 1 - 2*((qy*qy)+(qz*qz)))) * 180/PI;//roll
    filter.filter(altitude,acceleration);
    sat = gps.satellites.value();
    lat = gps.location.lat();
    lon = gps.location.lng();
    rx = gps.charsProcessed();
    filtered_acceleration = filter.update_state[2];
    filtered_altitude = filter.update_state[0];
    temp = bme.readTemperature();
    pressure = bme.readPressure() / 100.0;
    char message[300] = "";
    snprintf(message, sizeof(message), 
      "AngleX: %.2f, AngleY: %.2f, AngleZ: %.2f,Raw_Alt: %.2f, Filt_Alt: %.2f, Raw_Acc: %.2f, Filt_Acc: %.2f , qw: %.2f, qx: %.2f, qy: %.2f, qz: %.2f, lat: %.6f, lon: %.6f, sat: %d, char_rx: %.2f, temp: %.2f, pressure: %.2f, Stage: %d",
       angleX, angleY, angleZ, altitude, filtered_altitude, acceleration, filtered_acceleration, qw, qx, qy, qz, lat, lon, sat, rx, temp, pressure, stage_detector.stage);
    
    switch(stage_detector.stage){
      case 0:
        //PRELAUNCH
        digitalWrite(redLED, HIGH);
        digitalWrite(Buzzer,HIGH);
        if (millis() - lastSDWrite >= SDWriteInterval) {
          File dataFile = SD.open(filenameObj.c_str(), FILE_WRITE);
          logData(dataFile, acceleration, altitude, filtered_acceleration, filtered_altitude, speed, stage_detector.stage, running_time);
          dataFile.flush(); // Ensure data is written
          gps.encode(ss.read());
          lastSDWrite = millis();
        }
        transmit_data(message,rf95);
        //Serial.println("Prelaunch State");
        if(filtered_acceleration <= -20){//change with orientation, 0 because we have it opposite dir with gravity
            stage_detector.stage = 1;
        }
        break;
      case 1:
        //LAUNCHED, WAIT FOR BURNOUT
        /*
        if(tiltLock(angleX,angleY)){
          cutoff();
          stage_detector.stage = 6;
        }*/
        if (millis() - lastSDWrite >= SDWriteInterval) {
          File dataFile = SD.open(filenameObj.c_str(), FILE_WRITE);
          logData(dataFile, acceleration, altitude, filtered_acceleration, filtered_altitude, speed, stage_detector.stage, running_time);
          dataFile.flush(); // Ensure data is written
          gps.encode(ss.read());
          lastSDWrite = millis();
        }
        transmit_data(message,rf95);
        if(stage_detector.burnout(filtered_acceleration)){//change with orientation
            deploySeperation();// deploy pyro for seperation
            stage_detector.stage = 2;
        }
        break;
      case 2:
        //WAITING FOR APOGEE
        /*
        if(tiltLock(angleX,angleY)){
          cutoff();
          stage_detector.stage = 6;
        }*/
        if (millis() - lastSDWrite >= SDWriteInterval) {
          File dataFile = SD.open(filenameObj.c_str(), FILE_WRITE);
          logData(dataFile, acceleration, altitude, filtered_acceleration, filtered_altitude, speed, stage_detector.stage, running_time);
          dataFile.flush(); // Ensure data is written
          gps.encode(ss.read());
          lastSDWrite = millis();
        }
        transmit_data(message,rf95);
        if((stage_detector.apogee(filtered_altitude))){
          //APOGEE REACHED, AND ALTITUDE READ OVER 80% OF SIMULATED APOGEE
          deployS1drouge();//just turn on led here cause we missing stuff
          deployS1main();//deploy first stage main cause we just have main
          stage_detector.stage = 3;
          digitalWrite(redLED, LOW);
        }
        break;
      case 3:
        if (millis() - lastSDWrite >= SDWriteInterval) {
          File dataFile = SD.open(filenameObj.c_str(), FILE_WRITE);
          logData(dataFile, acceleration, altitude, filtered_acceleration, filtered_altitude, speed, stage_detector.stage, running_time);
          dataFile.flush(); // Ensure data is written
          gps.encode(ss.read());
          lastSDWrite = millis();
        }
        transmit_data(message,rf95);
        Serial.println("Main Chute deployed, waitting for landing");
        if(stage_detector.land(filtered_altitude)){
            stage_detector.stage = 4;
        }
        break;
      case 4:
        if (millis() - lastSDWrite >= SDWriteInterval) {
          File dataFile = SD.open(filenameObj.c_str(), FILE_WRITE);
          logData(dataFile, acceleration, altitude, filtered_acceleration, filtered_altitude, speed, stage_detector.stage, running_time);
          dataFile.flush(); // Ensure data is written
          gps.encode(ss.read());
          lastSDWrite = millis();
        }
        transmit_data(message,rf95);
        Serial.println("Land detected, turnon low power mode");
        bno.setMode(OPERATION_MODE_CONFIG);
        bno.enterSuspendMode();
        bno.setMode(OPERATION_MODE_NDOF);
        bno.setExtCrystalUse(false);
        bme.setSampling(Adafruit_BME280::MODE_SLEEP);
        dataFile.close();
        stage_detector.stage = 5;
        break;
      case 5:
        Serial.println("On low power mode");
        transmit_data(message,rf95);
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
      case 6:
        Serial.println("Angle go over safety, the goose is cooked! ");
        transmit_data(message,rf95);
        digitalWrite(greenLED, LOW);
        digitalWrite(redLED, LOW);
        digitalWrite(blueLED, LOW);
        cutoff();
    }
  /*
  Serial.print("Altitude: ");Serial.print(filtered_altitude);Serial.print(", ");
  Serial.print("Acceleration: ");Serial.print(filtered_acceleration);Serial.print(", ");
  Serial.print("angleX: ");Serial.print(angleX);Serial.print(", ");
  Serial.print("angleY: ");Serial.print(angleY);Serial.print(", ");
  Serial.print("angleZ: ");Serial.print(angleZ);Serial.print(", ");
  Serial.print("qw: ");Serial.print(qw);Serial.print(", ");
  Serial.print("qx: ");Serial.print(qx);Serial.print(", ");
  Serial.print("qy: ");Serial.print(qy);Serial.print(", ");
  Serial.print("qz: ");Serial.print(qz);Serial.print(", ");
  Serial.print("lat: %.6f"+lat);Serial.print(lat);Serial.print(", ");
  Serial.print("lon: %.6f"+lon);Serial.print(lon);Serial.print(", ");
  Serial.print("sat: ");Serial.print(sat);Serial.print(", ");
  Serial.print("chars prosess: ");Serial.print(rx);Serial.print(", ");
  Serial.print("fail check sum: "); Serial.print(gps.failedChecksum()); Serial.print(", ");
  Serial.print("Stage: "); Serial.println(stage_detector.stage);
  */
  Serial.println(message);
    
  //-------------
  
  }
}