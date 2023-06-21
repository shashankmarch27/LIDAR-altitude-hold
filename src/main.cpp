#include <Arduino.h> 
#include <LittleFS.h>
#include <SingleFileDrive.h>

#include <sbus.h>
#include <tfminis.h>
#include <pid.h>

#define PID_FREQUENCY 500

#define target_distance 100
#define buffer_distance 0

float KP = 3.0;
float KI = 1.2;
float KD = 0.1;

double b_distance = 0.96;
uint32_t distance =0;

double b_throttle = 0.95;
uint16_t throttle_prev = 0;

File logFile;

int current_millis;
int previous_millis;

bool plugged;
bool logging;
bool header_created;

sbus reciever(&Serial2);
tfminis lidar(&Serial1);
pid throttle(PID_FREQUENCY);

void myPlugCB(uint32_t data) {
  plugged = 1;
  logFile.close();
}

void myUnplugCB(uint32_t data) {
  plugged = 0;
}

void myDeleteCB(uint32_t data) {
  LittleFS.remove("file.csv");
}

void setup() {
  Serial.begin(9600);

  LittleFS.begin();
  singleFileDrive.onPlug(myPlugCB);
  singleFileDrive.onUnplug(myUnplugCB);
  singleFileDrive.onDelete(myDeleteCB);
  singleFileDrive.begin("file.csv", "logFile.csv");

  reciever.init();
  lidar.init();
}

void loop() {
  reciever.read();
  lidar.read();

  // check logging switch
  if(reciever.data[6] > 1500){
    logging = true;
  }
  else{
    logging = false; 
  }

  
  if(logging &&!plugged){

    if(!header_created){
      logFile = LittleFS.open("file.csv", "a");
      logFile.printf("Time,Altitude,Throttle,Kp,Ki,Kd\n");
      header_created = true;
    }
    
    distance = b_distance*distance + (1-b_distance)*(lidar.distance);
    if (abs(((int)distance)-target_distance) > buffer_distance)
    {
    reciever.data[2] =   map(throttle.compute(distance,target_distance,KP,KI,KD),0,1023,172,1810);
    reciever.data[2] = b_throttle*throttle_prev + (1.0-b_throttle)*reciever.data[2];
    reciever.data[2] =  constrain(reciever.data[2] , 172 , 1810);
    throttle_prev = reciever.data[2];
    }else{
    reciever.data[2] =   map(throttle.compute(distance,target_distance,KP,KI,0),0,1023,172,1810);
    reciever.data[2] = b_throttle*throttle_prev + (1.0-b_throttle)*reciever.data[2];
    reciever.data[2] =  constrain(reciever.data[2] , 172 , 1810);
    throttle_prev = reciever.data[2];
    }
    
    current_millis = millis();
    if(current_millis - previous_millis > 500){
      previous_millis = current_millis;
      logFile.printf("%d,%d,%d,%f,%f,%f\n",current_millis,distance,reciever.data[2],KP,KI,KD);
    }
  }

  else{
    if(header_created){
      logFile.close();
      header_created = false;
    }
    throttle.reset(map(reciever.data[2],172,1810,0,1023));
  }

  reciever.write();
}