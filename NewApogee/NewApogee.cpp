#include "NewApogee.h"

bool reached = false;
int count = 0;
double old_alt = 0;
double cur_alt = 0;

void detector::apogee(double reading_alt){
        cur_alt = reading_alt;
        if(cur_alt < old_alt){
            count++;
            old_alt = cur_alt;
            Serial.print("count: ");
            Serial.println(count);
        }
        else{
            count = 0;
            old_alt = cur_alt;
            Serial.print("count: ");
            Serial.println(count);
        }
        if(count >= 8){
            Serial.println("apogee reached");
            reached = true;
        }
    }