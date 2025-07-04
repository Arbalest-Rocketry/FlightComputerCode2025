#pragma once
#include <Arduino.h>
#include <utility/imumaths.h>
#include <Wire.h>
#include <SD.h>
#include <cmath>
#include <RH_RF95.h>


struct detector{
  const int WINDOW_SIZE = 21;
  double windowList[21];
  bool listFilled;
  int windowindex;
  double currentSum;
  double previousSum;
  double oldestAlt;
  double cur_alt;
  double old_alt;
  double new_alt;
  double cur_acc;
  int count;
  int stage;
  bool apogee(double& reading_alt);
  bool land(double& reading_alt);
  bool burnout(double& reading_acc);
  char stageNames[9];
};
void logData(File& dataFile, double rawAccel, double rawAltitude, double filteredAccel, double filteredAltitude, double speed,  int stage, long time);
void deployPyro(int pin);
void deployS1drouge();
void deployS1main();
void deployS2drouge();
void deployS2main();
void deploySeperation();
void deployIgnition();
void transmit_data(char message[], RH_RF95 rf95);
void initAngle(double angleX, double angleY);
bool tiltLock(double angleX, double angleY);
void cutoff();
String FileName();

