#include <Arduino.h>

#define HEADER_LIDAR 0X59
#define BAUDRATE_LIDAR 115200

#define HEADER_SBUS 0X0F
#define FOOTER_SBUS 0X00
#define BAUDRATE_SBUS 100000

bool header_detected_lidar = false;
bool header_detected_sbus = false;
bool frame_lost;
bool failsafe;

int prev_buffer_lidar;
int buffer_lidar;
int check_sum;
int distance;
int temperature;
int strength;

int prev_buffer_sbus;
int buffer_sbus;

int data_rx[25];
int data_tx[25];
int data_recieve[16];
int rx_index;

int data_lidar[9];
int lidar_index;

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

void initSbus(int rx_pin = 5, int tx_pin = 4){
  Serial2.setTX(tx_pin);
  Serial2.setRX(rx_pin);
  Serial2.begin(BAUDRATE_SBUS, SERIAL_8E2);
}

void readSbus(){
    while(Serial2.available()){
    prev_buffer_sbus = buffer_sbus;
    buffer_sbus = Serial2.read();

    if(header_detected_sbus == true){
      data_rx[rx_index] = buffer_sbus;
      rx_index++;
      if(rx_index > 23){
        header_detected_sbus = false;
      }
    }
    else{
      if(prev_buffer_sbus == FOOTER_SBUS && buffer_sbus == HEADER_SBUS){
      header_detected_sbus = true;
      data_rx[0] = 0x0F;
      data_rx[24] = 0x00;
      rx_index = 1;
      }
    }
    frame_lost = (data_rx[23] & 0x04) >> 2;
    failsafe = (data_rx[23] & 0x08) >> 3;
    if(!failsafe){
      data_recieve[0]  = (data_rx[1] | data_rx[2] << 8) & 0x07FF;
      data_recieve[1]  = ((data_rx[2] >> 3) | (data_rx[3] << 5)) & 0x07FF;
      data_recieve[2]  = ((data_rx[3] >> 6) | (data_rx[4] << 2) | (data_rx[5] << 10)) & 0x07FF;
      data_recieve[3]  = ((data_rx[5] >> 1) | (data_rx[6] << 7)) & 0x07FF;
      data_recieve[4]  = ((data_rx[6] >> 4) | (data_rx[7] << 4)) & 0x07FF;
      data_recieve[5]  = ((data_rx[7] >> 7) | (data_rx[8] << 1) | (data_rx[9] << 9)) & 0x07FF;
      data_recieve[6]  = ((data_rx[9] >> 2) | (data_rx[10] << 6)) & 0x07FF;
      data_recieve[7]  = ((data_rx[10] >> 5) | (data_rx[11] << 3)) & 0x07FF;
      data_recieve[8]  = ((data_rx[12] | (data_rx[13] << 8))) & 0x07FF;
      data_recieve[9]  = ((data_rx[13] >> 3) | (data_rx[14] << 5)) & 0x07FF;
      data_recieve[10] = ((data_rx[14] >> 6) | (data_rx[15] << 2) | (data_rx[16] << 10)) & 0x07FF;
      data_recieve[11] = ((data_rx[16] >> 1) | (data_rx[17] << 7)) & 0x07FF;
      data_recieve[12] = ((data_rx[17] >> 4) | (data_rx[18] << 4)) & 0x07FF;
      data_recieve[13] = ((data_rx[18] >> 7) | (data_rx[19] << 1) | (data_rx[20] << 9)) & 0x07FF;
      data_recieve[14] = ((data_rx[20] >> 2) | (data_rx[21] << 6)) & 0x07FF;
      data_recieve[15] = ((data_rx[21] >> 5) | (data_rx[22] << 3)) & 0x07FF;
    }
    else{
      data_recieve[0]  = 992;
      data_recieve[1]  = 992;
      data_recieve[2]  = 150;
      data_recieve[3]  = 992;
      data_recieve[4]  = 992;
      data_recieve[5]  = 992;
      data_recieve[6]  = 992;
      data_recieve[7]  = 992;
      data_recieve[8]  = 992;
      data_recieve[9]  = 0;
      data_recieve[10] = 0;
      data_recieve[11] = 0;
      data_recieve[12] = 0;
      data_recieve[13] = 0;
      data_recieve[14] = 0;
      data_recieve[15] = 0;
    }
  }
}

void writeSbus(){
  data_tx[0] = HEADER_SBUS;
  data_tx[1] = (data_recieve[0]) & 0xFF;
  data_tx[2] = ((data_recieve[0] ) >> 8 | (data_recieve[1] ) << 3) & 0xFF;
  data_tx[3] = ((data_recieve[1] ) >> 5 | (data_recieve[2] ) << 6) & 0xFF;
  data_tx[4] = ((data_recieve[2] ) >> 2) & 0xFF;
  data_tx[5] = ((data_recieve[2] ) >> 10 | (data_recieve[3] ) << 1) & 0xFF;
  data_tx[6] = ((data_recieve[3] ) >> 7 | (data_recieve[4] ) << 4) & 0xFF;
  data_tx[7] = ((data_recieve[4] ) >> 4 | (data_recieve[5] ) << 7) & 0xFF;
  data_tx[8] = ((data_recieve[5] ) >> 1) & 0xFF;
  data_tx[9] = ((data_recieve[5] ) >> 9  | (data_recieve[6] ) << 2) & 0xFF;
  data_tx[10] = ((data_recieve[6] ) >> 6  | (data_recieve[7] ) << 5) & 0xFF;
  data_tx[11] = ((data_recieve[7] ) >> 3) & 0xFF;
  data_tx[12] = ((data_recieve[8] )) & 0xFF;
  data_tx[13] = ((data_recieve[8] ) >> 8 | (data_recieve[9]  ) << 3) & 0xFF;
  data_tx[14] = ((data_recieve[9] ) >> 5 | (data_recieve[10] ) << 6) & 0xFF;
  data_tx[15] = ((data_recieve[10] ) >> 2) & 0xFF;
  data_tx[16] = ((data_recieve[10] ) >> 10 | (data_recieve[11] ) << 1) & 0xFF;
  data_tx[17] = ((data_recieve[11] ) >> 7  | (data_recieve[12] ) << 4) & 0xFF;
  data_tx[18] = ((data_recieve[12] ) >> 4  | (data_recieve[13] ) << 7) & 0xFF;
  data_tx[19] = ((data_recieve[13] ) >> 1) & 0xFF;
  data_tx[20] = ((data_recieve[13] ) >> 9  | (data_recieve[14] ) << 2) & 0xFF;
  data_tx[21] = ((data_recieve[14] ) >> 6  | (data_recieve[15] ) << 5) & 0xFF;
  data_tx[22] = ((data_recieve[15] ) >> 3) & 0xFF;
  data_tx[24] = FOOTER_SBUS;

  for(int tx_index = 0; tx_index < 25;tx_index++){
    Serial.write(data_tx[tx_index]);
  }
}

void setup() {
  initLidar();
  initSbus();
  Serial.begin(9600);
}

void loop() {
  readSbus();
  Serial.println(data_recieve[2]);
}