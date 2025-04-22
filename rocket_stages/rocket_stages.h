#pragma once
#include <Arduino.h>
#define THRESHOLD 8;
struct detector{
  double cur_alt;
  double old_alt;
  double cur_acc;
  int count;
  int stage;
  bool apogee(double& reading_alt);
  bool land(double& reading_alt);
  bool burnout(double& reading_acc);
};
void state_machine(detector& detector, double acceleration, double altitude);