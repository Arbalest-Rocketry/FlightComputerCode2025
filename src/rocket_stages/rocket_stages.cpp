#include "rocket_stages.h"

int count = 0;
double old_alt = 0;
double cur_alt = 0;
double cur_acc = 0;
int stage = 0;
const int S1drouge, S1main, S2drouge, S2main, seperation, ignite;
const char* stageNames[9] = {"PRE_LAUNCH",
                      "LAUNCH_DETECTED",
                      "FIRST_BURNOUT",
                      "IGNITE",
                      "SECOND_BURNOUT",
                      "APOGEE_REACHED",
                      "MAIN_CHUTE_DEPLOYED",
                      "LANDING_DETECTED",
                      "LOW_POWER_MODE"};

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
    if(abs(cur_alt - old_alt)<0.1){
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
void logData(File& dataFile, double rawAccel, double rawAltitude, double filteredAccel, double filteredAltitude, int stage, long time){
  dataFile = SD.open("FLIGHTDATA.txt", FILE_WRITE);
  if(dataFile){
    dataFile.print("t: ");dataFile.print(time);dataFile.print(", ");
    dataFile.print("raw_accel: ");dataFile.print(rawAccel);dataFile.print(", ");
    dataFile.print("raw_alt: ");dataFile.print(rawAltitude);dataFile.print(", ");
    dataFile.print("filt_accel: ");dataFile.print(filteredAccel);dataFile.print(", ");
    dataFile.print("filt_alt: ");dataFile.print(filteredAltitude);dataFile.print(", ");
    dataFile.print("stage: ");dataFile.println(stage);
    dataFile.close();
    Serial.println("logged");

  }
  else{
    Serial.println("Error, check card PLSSSSSSSSSSS");
  }

}
void deployPyro(int pin, const char* message){
  Serial.println(message);
  digitalWrite(pin, HIGH);
  delay(5000);
  digitalWrite(pin, LOW);
}
void deployIgnition(){deployPyro(ignite, "Deploying Ignite Pyro!");}
void deployS1drouge(){deployPyro(S1drouge, "Deploy Stage 1 drouge!");}
void deployS1main(){deployPyro(S1main, "Deploy Stage 1 main!");}
void deployS2drouge(){deployPyro(S2drouge, "Deploy Stage 2 drouge!");}
void deployS2main(){deployPyro(S2main, "Deploy Stage 2 main!");}
void deploySeperation(){deployPyro(seperation, "Deploy seperation!");}

void transmit_data(){
  //transmitting code go here
}

