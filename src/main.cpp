#include <Arduino.h> 
#include <LittleFS.h>
#include <SingleFileDrive.h>

#include <sbus.h>
#include <tfminis.h>
#include <pid.h>

#define SBUS_MINIMUM 172
#define SBUS_MAXIMUM 1810
#define UINT10_MINIMUM 0
#define UINT10_MAXIMUM 1023

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

bool plugged = false;
bool logging = false;
bool header_created = false;

// object for sbus reciever, lidar, pid
sbus reciever(&Serial2);
tfminis lidar(&Serial1);
pid throttle(PID_FREQUENCY);

int SBUS_TO_UINT10(int value){
  return map(value,SBUS_MINIMUM,SBUS_MAXIMUM,UINT10_MINIMUM,UINT10_MAXIMUM);
}

int UINT10_TO_SBUS(int value){
  return map(value,UINT10_MINIMUM,UINT10_MAXIMUM,SBUS_MINIMUM,SBUS_MAXIMUM);
}

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
  LittleFS.begin();
  singleFileDrive.onPlug(myPlugCB);
  singleFileDrive.onUnplug(myUnplugCB);
  singleFileDrive.onDelete(myDeleteCB);
  singleFileDrive.begin("file.csv", "logFile.csv");

  // initialize sbus communication
  reciever.init();
  // initialize lidar communication
  lidar.init();
}

void loop() {
  // read reciever data
  reciever.read();
  // read lidar data
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
    else{
      distance = b_distance*distance + (1-b_distance)*(lidar.distance);

      if (abs(((int)distance)-target_distance) > buffer_distance){
        reciever.data[THROTTLE] = UINT10_TO_SBUS(throttle.compute(distance,target_distance,KP,KI,KD));
        reciever.data[THROTTLE] = b_throttle*throttle_prev + (1.0-b_throttle)*reciever.data[2];
        reciever.data[THROTTLE] = constrain(reciever.data[2] , 172 , 1810);
        throttle_prev = reciever.data[2];
      }
      else{
        reciever.data[THROTTLE] = UINT10_TO_SBUS(throttle.compute(distance,target_distance,KP,KI,0));
        reciever.data[THROTTLE] = b_throttle*throttle_prev + (1.0-b_throttle)*reciever.data[2];
        reciever.data[THROTTLE] = constrain(reciever.data[2] , 172 , 1810);
        throttle_prev = reciever.data[2];
      }

      current_millis = millis();
      if(current_millis - previous_millis > 500){
        previous_millis = current_millis;
        logFile.printf("%d,%d,%d,%f,%f,%f\n",current_millis,distance,reciever.data[THROTTLE],KP,KI,KD);
      }
    }
  }
  else{
    if(header_created){
      logFile.close();
      header_created = false;
    }
    throttle.reset(SBUS_TO_UINT10(reciever.data[THROTTLE]));
  }

  reciever.write();
}