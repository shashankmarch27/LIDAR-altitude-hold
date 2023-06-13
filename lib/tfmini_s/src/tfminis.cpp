#include "tfminis.h"

tfminis::tfminis(SerialUART *port, int tx, int rx){
    tfmini_s_port = port;
    tx_pin = tx;
    rx_pin = rx;
}

void tfminis::init(){
    tfmini_s_port->setRX(rx_pin);
    tfmini_s_port->setTX(tx_pin);
    tfmini_s_port->begin(BAUDRATE_LIDAR, SERIAL_8N1);
}

void tfminis::read(){
    while(tfmini_s_port->available()){
    prev_buffer_lidar = buffer_lidar;
    buffer_lidar = tfmini_s_port->read();

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