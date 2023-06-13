#include <Arduino.h>

#include <sbus.h>
#include <tfminis.h>
#include <pid.h>

#define KP 0
#define KI 0
#define KD 0

sbus reciever(&Serial2,4,5);
tfminis lidar(&Serial1,0,1);
pid throttle;

void setup() {
  reciever.init();
  lidar.init();
  Serial.begin(9600);
}

void loop() {

// read the sbus packed
  reciever.read();

// run pid loop at 2000hz or every 500 us
    reciever.data[2] = map(throttle.compute(0, 100, KP,KI,KD), 0, 1023, 172, 1810);

    Serial.println(reciever.data[2]);
// send sbus packet
    reciever.write();
}