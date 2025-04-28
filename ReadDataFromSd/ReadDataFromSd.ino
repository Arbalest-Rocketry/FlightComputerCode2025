#include <SD.h>

const int chipSelect = 10;

File ReadFile;

String dataLine = "";

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);

  while(!Serial);

  Serial.print("Initializing SD card...");

  if (!SD.begin(chipSelect)) {
    Serial.println("initialization failed. Things to check:");
    while (true); //halt
  }
  Serial.println("initialization done.");

  //Reading Here
  ReadFile = SD.open("FLIGHTDATA.txt", FILE_READ);
  while(ReadFile.available()){
    Serial.write(ReadFile.read());
  }
  
  ReadFile.close();
  //------------------------------------------------
  
}

void loop() {
  // put your main code here, to run repeatedly:

}
