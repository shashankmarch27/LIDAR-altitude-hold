#include <Arduino.h>

#include <sbus.h>
#include <tfminis.h>
#include <pid.h>

#define KP 2.0
#define KI 0.4
#define KD 0.4

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
    reciever.data[2] = map(throttle.compute(lidar.distance,100,KP,KI,KD),0,1023,172,1810);
  }
  else{
    throttle.reset();
  }
  Serial.print(reciever.data[0]);
  Serial.print("   ");
  Serial.print(reciever.data[1]);
  Serial.print("   ");
  Serial.print(reciever.data[2]);
  Serial.print("   ");
  Serial.print(reciever.data[3]);
  Serial.print("   ");
  Serial.print(reciever.data[4]);
  Serial.print("   ");
  Serial.print(reciever.data[5]);
  Serial.print("   ");
  Serial.println(reciever.data[6]);
  reciever.write();
}