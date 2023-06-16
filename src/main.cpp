#include <Arduino.h>

#include <sbus.h>
#include <tfminis.h>
#include <pid.h>

#define PID_FREQUENCY 2000
#define KP 1.2
#define KI 0.5
#define KD 0.1

sbus reciever(&Serial2);
tfminis lidar(&Serial1);
pid throttle(PID_FREQUENCY);

void setup() {
  reciever.init();
  lidar.init();
  Serial.begin(9600);
}

void loop() {
  reciever.read();
  lidar.read();
  if(reciever.data[6] > 1500){
    reciever.data[2] = map(throttle.compute(lidar.distance,100,KP,KI,KD),0,1023,172,1810);
  }
  else{
    throttle.reset();
    throttle.integral_value = map(reciever.data[2],172,1810,0,1023);
  }
  Serial.println(reciever.data[2]);
  reciever.write();
}