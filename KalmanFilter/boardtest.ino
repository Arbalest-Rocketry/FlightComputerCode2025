#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include <SPI.h>
#include <Adafruit_BMP280.h>
#include <KalmanFilter.h>
#include <SD.h>

Adafruit_BMP280 bmp;
Adafruit_BNO055 bno = Adafruit_BNO055();
KalmanFilter filter;
const int chipSelect = 10;
File TestFile;

double ac, s;

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
  SD.remove("TEST.txt");

  imu::Vector<3> accel = bno.getVector(Adafruit_BNO055::VECTOR_ACCELEROMETER);
  ac = accel.z();
  s = bmp.readAltitude(1012.9);
  filter.initial(s,ac);
  TestFile = SD.open("TEST.txt", FILE_WRITE);
  TestFile.print(String("time") + ", ");
  TestFile.print(String("raw_a") + ", ");
  TestFile.print(String("raw_alt") + ", ");
  TestFile.print(String("fil_a") + ", ");
  TestFile.print(String("fil_alt") + ", ");
  TestFile.println(String("vel"));
  TestFile.close();

  while(millis()< 60000){
    imu::Vector<3> accel = bno.getVector(Adafruit_BNO055::VECTOR_ACCELEROMETER);
    ac = accel.z();
    s = bmp.readAltitude(1012.9);
    filter.filter(s,ac);
    if(millis()> 10000){
      
      TestFile = SD.open("TEST.txt", FILE_WRITE);
      
      if(TestFile){
        TestFile.print(String(millis()) + ", ");
        TestFile.print(String(ac) + ", ");
        TestFile.print(String(s) + ", ");
        TestFile.print(String(filter.current_state[2]) + ", ");
        TestFile.print(String(filter.current_state[0]) +  ", ");
        TestFile.println(filter.current_state[1]);
        TestFile.close();
        Serial.println("Added");
      }
      else{
        Serial.print("can't access");
      }
      
    }
   
  }
  Serial.println("Done");
}

void loop() {
  // put your main code here, to run repeatedly:

}
