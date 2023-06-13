#include <Arduino.h>
#include <sbus.h>

#define HEADER_LIDAR 0X59
#define BAUDRATE_LIDAR 115200

#define KP 1
#define KI 1
#define KD 1

bool header_detected_lidar = false;

int prev_buffer_lidar;
int buffer_lidar;
int check_sum;
int distance;
int temperature;
int strength;


int data_lidar[9];
int lidar_index;

int current_millis;
int previous_millis_pid;
int previous_millis_sbus;

int error;
int prev_error;
float proportional_value;
float integral_value;
float differential_value;

sbus reciever(&Serial2,4,5);

void initLidar(int rx_pin = 1,int tx_pin = 0){
  Serial1.setRX(rx_pin);
  Serial1.setTX(tx_pin);
  Serial1.begin(BAUDRATE_LIDAR, SERIAL_8N1);
}

void readLidar(){
  while(Serial1.available()){
    prev_buffer_lidar = buffer_lidar;
    buffer_lidar = Serial1.read();

    if(header_detected_lidar){
      data_lidar[lidar_index] = buffer_lidar;
      lidar_index++;
      if(lidar_index > 8){
        header_detected_lidar = false;
      }
    }
    else{
      if(prev_buffer_lidar == HEADER_LIDAR && buffer_lidar == HEADER_LIDAR){
        header_detected_lidar = true;
        data_lidar[0] = HEADER_LIDAR;
        data_lidar[1] = HEADER_LIDAR; 
        lidar_index = 2;
      }
    }
  }
  check_sum = (data_lidar[0] + data_lidar[1] + data_lidar[2] + data_lidar[3] + data_lidar[4] + data_lidar[5] + data_lidar[6] + data_lidar[7] + data_lidar[8]) & 0xFF;

    if(check_sum == data_lidar[8]){
      strength = data_lidar[4] + (data_lidar[5] << 8);
      temperature = (data_lidar[6] + (data_lidar[7] << 8))/8 -256;
      if(strength >100){
        distance = data_lidar[2] + (data_lidar[3] << 8);
      }
    }
    else{
      header_detected_lidar = false;
    }
}

int pidCompute(int current_value, int target_value, float kp, float ki, float kd, float time){
  prev_error = error;
  error = target_value - current_value;

  proportional_value = kp * error * time;
  integral_value += ki * error * time;
  integral_value = constrain(integral_value, 0, 1023);
  differential_value = kd * (error - prev_error) * time;

  return constrain(proportional_value + integral_value + differential_value, 0, 1023);
}

void setup() {
  reciever.init();

  initLidar();
  Serial.begin(9600);
}

void loop() {

// read the sbus packed
  reciever.read();

// run pid loop at 2000hz or every 500 us
  current_millis = micros();
  if(current_millis - previous_millis_pid > 500 && reciever.data[6] > 1500){
    previous_millis_pid = current_millis;
    reciever.data[2] = map(pidCompute(distance, 100, KP,KI,KD,(current_millis - previous_millis_pid) * 0.000001 ), 0, 1023, 172, 1810);
  }

// send sbus packet
    reciever.write();
}