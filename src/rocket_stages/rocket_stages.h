#pragma once
#include <Arduino.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include <Adafruit_BMP280.h>
#include <Wire.h>
#include <SD.h>
#include <math.h>
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
void transmit_data();

