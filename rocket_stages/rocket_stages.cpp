#include "rocket_stages.h"

int count = 0;
double old_alt = 0;
double cur_alt = 0;
double cur_acc = 0;
int stage = 0;

bool detector::apogee(double& reading_alt){
        cur_alt = reading_alt;
        if(count >= 8){
            Serial.println("apogee reached");
            count = 0;
            return true;
        }
        if(cur_alt < old_alt){
            count++;
            old_alt = cur_alt;
            Serial.print("count: ");
            Serial.println(count);
            return false;
        }
        else{
            count = 0;
            old_alt = cur_alt;
            Serial.print("count: ");
            Serial.println(count);
            return false;
        }
        
}
bool detector::land(double &reading_alt){
    cur_alt = reading_alt;
    if(count >=4){
        count = 0;
        return true;
    }
    if((cur_alt - old_alt)<0.1){
        count++;
        return false;
    }
    else{
        count = 0;
        old_alt = cur_alt;
        return false;
    }
}
bool detector::burnout(double &reading_acc){
    cur_acc = reading_acc;
    if (count >=3){
        return true;
    }
    if(cur_acc < 1.5){
        count++;
        return false;
    }
    else{
        count = 0;
        return false;
    }
    
}
void state_machine(detector& detector, double acceleration, double altitude){
    switch(detector.stage){
        case 0:
            Serial.println("Prelaunch State");
            if(acceleration >= 8){//change with orientation
                detector.stage = 1;
            }
            break;
        case 1:
            Serial.println("Launch Detected, wating for 1st burnout");
            if(detector.burnout(acceleration)){//change with orientation
                detector.stage = 2;
            }
            break;
        case 2:
            Serial.println("Burnout detected, waiting for seperation");
            //pyro turn on to separate the stage
            delay(6000);//wait 6 seconds then change to ignite stage
            if(acceleration>=5){
                detector.stage = 3;
            }
            break;
        case 3:
            Serial.println("Igniting, wait for second burnout");
            if(detector.burnout(acceleration)){
                detector.stage = 4;
            }
            break;
        case 4:
            Serial.println("Second burnout detected, waiting for apogee");
            if(detector.apogee(altitude)){
                detector.stage = 5;
            }
            break;
        case 5:
            Serial.println("Apogee detected, wait for main chute deploy");
            if(altitude <= 457.2){
                detector.stage = 6;
            }
            break;
        case 6:
            Serial.println("Main Chute deployed, waitting for landing");
            if(detector.land(altitude)){
                detector.stage = 7;
            }
            break;
        case 7:
            Serial.println("Land detected, turnon low power mode");
            detector.stage = 8;
            break;
        case 8:
            Serial.println("On low power mode");
            break;
    }
}