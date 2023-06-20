#include <Arduino.h> 
#include <LittleFS.h>
#include <SingleFileDrive.h>
#include <hardware/vreg.h>

#include <sbus.h>
#include <tfminis.h>
#include <pid.h>

#define PID_FREQUENCY 2000
float KP = 1.2;
float KI = 0.5;
float KD = 0.1;

int current_millis;
int previous_millis;

bool plugged;
bool logging;
bool armed;
bool header_created;

sbus reciever(&Serial2);
tfminis lidar(&Serial1);
pid throttle(PID_FREQUENCY);

void myPlugCB(uint32_t data) {
  plugged = 1;
}

void myUnplugCB(uint32_t data) {
  plugged = 0;
}

void myDeleteCB(uint32_t data) {
  LittleFS.remove("file.csv");
}

void setup() {
   vreg_set_voltage(VREG_VOLTAGE_1_10);
  LittleFS.begin();
  singleFileDrive.onPlug(myPlugCB);
  singleFileDrive.onUnplug(myUnplugCB);
  singleFileDrive.onDelete(myDeleteCB);
  singleFileDrive.begin("file.csv", "log.csv");

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

  // check arm status
  if(reciever.data[5] > 800){
    if(reciever.data[4] > 1500){
      armed = true;
    }
  }
  else{
    armed = false;
  }

  if(logging && armed && !plugged){

    if(!header_created){
      File log = LittleFS.open("file.csv", "a");
      log.printf("Time,Altitude,Throttle,Kp,Ki,Kd\n");
      log.close();
      header_created = true;
    }

    reciever.data[2] = map(throttle.compute(lidar.distance,100,KP,KI,KD),0,1023,172,1810);
    
    current_millis = micros();
    if(current_millis - previous_millis > 500){
      previous_millis = current_millis;
      File log = LittleFS.open("file.csv", "a");
      log.printf("%d,%d,%d,%f,%f,%f\n",current_millis,lidar.distance,reciever.data[2],KP,KI,KD);
      log.close();
    }
  }
  else{
    throttle.reset();
    throttle.integral_value = map(reciever.data[2],172,1810,0,1023);
  }

  reciever.write();
}