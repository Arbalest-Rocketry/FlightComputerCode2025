#pragma once
#include <Arduino.h>

#include <Adafruit_BMP280.h> // change to appopriate sensor
#include <Adafruit_Sensor.h>

#define WINDOW_SIZE 21
#define THRESHOLD 8
extern bool reached;

typedef struct {
    double *backing_array;
    size_t capacity;
    size_t size;
    size_t front;
    double sum_of_elements;
} RollingWindow;

typedef struct {
    RollingWindow altitude_window;
    double last_altitude;
    int apogee_reached;
    int decrease_count; // Track consecutive decreases
} ApogeeDetector;

extern Adafruit_BMP280 bmp;

void init_rw(RollingWindow *rw, double *BackingArray, size_t capacity);
size_t mod_rw(size_t index, size_t modulo);
void add_data(RollingWindow *rw, double data);
double get_earliest_data(RollingWindow *rw);
double get_latest_data(RollingWindow *rw);

void init_apogee_detector(ApogeeDetector *detector, double *BackingArray, size_t capacity);
void update_apogee_detector(ApogeeDetector *detector, double cur_altitude);
int apogee_reached(ApogeeDetector *detector);