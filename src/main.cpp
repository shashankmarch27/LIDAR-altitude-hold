#include <Arduino.h>

#include <sbus.h>
#include <tfminis.h>
#include <pid.h>

#define KP 2.4
#define KI 1
#define KD 1.44 //KD = KP*KP*KI /4;

sbus reciever(&Serial2);
tfminis lidar(&Serial1);
pid throttle;

void setup() {
  reciever.init();
  lidar.init();
  Serial.begin(9600);
}

void loop() {
  reciever.read();
  if(reciever.data[6] > 1500){
    lidar.distance = map(reciever.data[0],172,992,0,100);
    reciever.data[2] = map(throttle.compute(lidar.distance,100,KP,KI,KD),0,1023,172,1810);
  }
  else{
    throttle.reset();
  }
  Serial.print(reciever.data[2]);
  Serial.print("   ");
  Serial.println(lidar.distance);
  reciever.write();
}