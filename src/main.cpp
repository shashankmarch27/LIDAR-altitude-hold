#include <Arduino.h>
#include <LittleFS.h>
#include <SingleFileDrive.h>

#include <sbus.h>
#include <tfminis.h>
#include <pid.h>

#define PID_FREQUENCY 2000
#define KP 1.2
#define KI 0.5
#define KD 0.1

bool plugged;

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
  if(reciever.data[6] > 1500){
    reciever.data[2] = map(throttle.compute(lidar.distance,100,KP,KI,KD),0,1023,172,1810);

    if(!plugged){
      File log = LittleFS.open("file.csv", "a");
      log.printf("%d,%d\n",lidar.distance,reciever.data[2]);
      log.close();
      delay(100);
    }
  }
  else{
    throttle.reset();
    throttle.integral_value = map(reciever.data[2],172,1810,0,1023);
  }

  reciever.write();
}