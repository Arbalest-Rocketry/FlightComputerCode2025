#pragma once
#include <Arduino.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include <Adafruit_BMP280.h>
#include <Wire.h>
#include <SD.h>
#include <cmath>
#include <RH_RF95.h>


struct detector{
  double cur_alt;
  double old_alt;
  double cur_acc;
  int count;
  int stage;
  bool apogee(double& reading_alt);
  bool land(double& reading_alt);
  bool burnout(double& reading_acc);
  char stageNames[9];
};
void logData(File& dataFile, double rawAccel, double rawAltitude, double filteredAccel, double filteredAltitude, int stage, long time);
void deployPyro(int pin);
void deployS1drouge();
void deployS1main();
void deployS2drouge();
void deployS2main();
void deploySeperation();
void deployIgnition();
void transmit_data(double AngleX, double AngleY, double Raw_Alt, double Filt_Alt, double Raw_Acc, double Filt_Acc, int Stage, RH_RF95 rf95);
void initAngle(double angleX, double angleY);
bool tiltLock(double angleX, double angleY);
void cutoff();
String FileName();

