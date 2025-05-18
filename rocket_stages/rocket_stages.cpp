#include "rocket_stages.h"

int count = 0;
double old_alt = 0;
double cur_alt = 0;
double cur_acc = 0;
int stage = 0;
int initialX, initialY;
const int S1drouge, S1main, S2drouge, S2main, seperation, ignite;

const int WINDOW_SIZE = 8;
double windowList[WINDOW_SIZE];
bool listFilled = false;
int index = 0;
int currentSum = 0;
int previousSum = 0;
double oldestAlt = 0;

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
/*
*I may change to this for apogee later
bool detector::apogee(double& reading_alt){
    if(!listFilled){
        windowList[index] = reading_alt;
        currentSum += reading_alt;
        index++;
        if(index == WINDOW_SIZE){
            listFilled = true;
            index = 0;
        }
    }
    else{
        oldestAlt = windowList[index];

        previousSum = currentSum - oldestAlt;
        currentSum = previousSum + reading_alt;

        windowList[index] = reading_alt;
        index = (index+1)%8;
        if(currentSum < previousSum){
            count++;
        }
        else{
            count = 0;
        }
        if(count>=8){
            return true;
        }
        else{
            return false;
        }
    }
}
*/
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
    if(cur_acc > 8){
        //larger than 8 because 9 is gravity
        count++;
        return false;
    }
    else{
        count = 0;
        return false;
    }
    
}
void logData(File& dataFile, double rawAccel, double rawAltitude, double filteredAccel, double filteredAltitude, int stage, long time){
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


void initAngle(double angleX, double angleY){
    initialX = angleX;
    initialY = angleY;
}


bool tiltLock(double angleX, double angleY){
    //angle will be change
    if((abs((angleY-initialY)>15))){
        Serial.println("WE COOKED GUYS!!");
        return true;
    }
    if(abs((angleX - initialX )>15)){
        Serial.println("WE COOKED GUYS!!");
        return true;
    }
    else{return false;}
}

void cutoff(){
    pinMode(S1drouge, OUTPUT);
    pinMode(S1main, OUTPUT);
    pinMode(S2drouge, OUTPUT);
    pinMode(S2main, OUTPUT);
    pinMode(seperation, OUTPUT);
    pinMode(ignite, OUTPUT);
}

void transmit_data(double AngleX, double AngleY, double Raw_Alt, double Filt_Alt, double Raw_Acc, double Filt_Acc, int Stage, RH_RF95 rf95){
  //transmitting code go here
    char message[120];
    snprintf(message, sizeof(message),
        "AngleX: %.2f, AngleY: %.2f, Raw_Alt: %.2f, Filt_Alt: %.2f, Raw_Acc: %.2f, Filt_Acc: %.2f , Stage: %d",
        AngleX, AngleY, Raw_Alt,
        Filt_Alt, Raw_Acc, Filt_Acc, Stage);
    rf95.send((uint8_t *)message, strlen(message));
    //rf95.waitPacketSent();
    Serial.println("✅ Telemetry Sent!");

  

    

}
String FileName() {
    String fileName;
      String OGfile = "FLIGHTDATA";
  
      for (int i = 1; i < 2000; i++) {
        
      if (!SD.exists((String(OGfile) + String(i)).c_str()) && SD.exists(String(OGfile).c_str())) {
           fileName = String(OGfile) + String(i);
          break;
  
      } else {
         fileName = OGfile; 
      }   
  }
  return fileName;
  }

