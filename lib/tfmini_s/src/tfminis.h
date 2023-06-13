#ifndef TFMINIS_H
#define TFMINIS_H

#include <Arduino.h>

#define HEADER_LIDAR 0X59
#define BAUDRATE_LIDAR 115200

class tfminis{

private:
    int tx_pin;
    int rx_pin;
    SerialUART *tfmini_s_port;

    bool header_detected_lidar = false;
    int prev_buffer_lidar;
    int buffer_lidar;
    int check_sum;
    int data_lidar[9];
    int lidar_index;

public:
    int distance;
    int temperature;
    int strength;
    
    tfminis(SerialUART *port, int tx, int rx);

    void init();
    void read();

};

#endif