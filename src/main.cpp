#include <Arduino.h>

#include <sbus.h>
#include <tfminis.h>
#include <pid.h>

#define PID_FREQUENCY 2000
#define KP 1.5
#define KI 2.123 //KI = 5/(3.14 * KP)
#define KD 0.3 //KD = KP*KP*KI/4

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
  }

  Serial.println(reciever.data[2]);
  reciever.write();
}