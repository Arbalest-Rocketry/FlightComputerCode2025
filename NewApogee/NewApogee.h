#pragma once
#include <Arduino.h>
#define THRESHOLD 8;
struct detector{
  double cur_alt;
  double old_alt;
  int count;
  bool reached;
  void apogee(double reading_alt);
};