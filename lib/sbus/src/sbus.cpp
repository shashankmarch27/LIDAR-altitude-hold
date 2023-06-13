#include "sbus.h"

sbus::sbus(SerialUART *port, int tx, int rx){
    sbus_port = port;
    tx_pin = tx;
    rx_pin = rx;
}

void sbus::init(){
    sbus_port->setTX(tx_pin);
    sbus_port->setRX(rx_pin);
    sbus_port->begin(BAUDRATE_SBUS,SERIAL_8E2);
}

void sbus::read(){
    while(sbus_port->available()){
        prev_buffer_sbus = buffer_sbus;
        buffer_sbus = sbus_port->read();

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
        
    data[0]  = (data_rx[1] | data_rx[2] << 8) & 0x07FF;
    data[1]  = ((data_rx[2] >> 3) | (data_rx[3] << 5)) & 0x07FF;
    data[2]  = ((data_rx[3] >> 6) | (data_rx[4] << 2) | (data_rx[5] << 10)) & 0x07FF;
    data[3]  = ((data_rx[5] >> 1) | (data_rx[6] << 7)) & 0x07FF;
    data[4]  = ((data_rx[6] >> 4) | (data_rx[7] << 4)) & 0x07FF;
    data[5]  = ((data_rx[7] >> 7) | (data_rx[8] << 1) | (data_rx[9] << 9)) & 0x07FF;
    data[6]  = ((data_rx[9] >> 2) | (data_rx[10] << 6)) & 0x07FF;
    data[7]  = ((data_rx[10] >> 5) | (data_rx[11] << 3)) & 0x07FF;
    data[8]  = ((data_rx[12] | (data_rx[13] << 8))) & 0x07FF;
    data[9]  = ((data_rx[13] >> 3) | (data_rx[14] << 5)) & 0x07FF;
    data[10] = ((data_rx[14] >> 6) | (data_rx[15] << 2) | (data_rx[16] << 10)) & 0x07FF;
    data[11] = ((data_rx[16] >> 1) | (data_rx[17] << 7)) & 0x07FF;
    data[12] = ((data_rx[17] >> 4) | (data_rx[18] << 4)) & 0x07FF;
    data[13] = ((data_rx[18] >> 7) | (data_rx[19] << 1) | (data_rx[20] << 9)) & 0x07FF;
    data[14] = ((data_rx[20] >> 2) | (data_rx[21] << 6)) & 0x07FF;
    data[15] = ((data_rx[21] >> 5) | (data_rx[22] << 3)) & 0x07FF;
    frame_lost = (data_rx[23] & 0x04) >> 2;
    failsafe = (data_rx[23] & 0x08) >> 3;
    }
}

void sbus::write(){
    current_millis = micros();
    if(current_millis - previous_millis > 6000 && sbus_port->available()){
        previous_millis = current_millis;
        data_tx[0] = HEADER_SBUS;
        data_tx[1] = (data[0]) & 0xFF;
        data_tx[2] = ((data[0] ) >> 8 | (data[1] ) << 3) & 0xFF;
        data_tx[3] = ((data[1] ) >> 5 | (data[2] ) << 6) & 0xFF;
        data_tx[4] = ((data[2] ) >> 2) & 0xFF;
        data_tx[5] = ((data[2] ) >> 10 | (data[3] ) << 1) & 0xFF;
        data_tx[6] = ((data[3] ) >> 7 | (data[4] ) << 4) & 0xFF;
        data_tx[7] = ((data[4] ) >> 4 | (data[5] ) << 7) & 0xFF;
        data_tx[8] = ((data[5] ) >> 1) & 0xFF;
        data_tx[9] = ((data[5] ) >> 9  | (data[6] ) << 2) & 0xFF;
        data_tx[10] = ((data[6] ) >> 6  | (data[7] ) << 5) & 0xFF;
        data_tx[11] = ((data[7] ) >> 3) & 0xFF;
        data_tx[12] = ((data[8] )) & 0xFF;
        data_tx[13] = ((data[8] ) >> 8 | (data[9]  ) << 3) & 0xFF;
        data_tx[14] = ((data[9] ) >> 5 | (data[10] ) << 6) & 0xFF;
        data_tx[15] = ((data[10] ) >> 2) & 0xFF;
        data_tx[16] = ((data[10] ) >> 10 | (data[11] ) << 1) & 0xFF;
        data_tx[17] = ((data[11] ) >> 7  | (data[12] ) << 4) & 0xFF;
        data_tx[18] = ((data[12] ) >> 4  | (data[13] ) << 7) & 0xFF;
        data_tx[19] = ((data[13] ) >> 1) & 0xFF;
        data_tx[20] = ((data[13] ) >> 9  | (data[14] ) << 2) & 0xFF;
        data_tx[21] = ((data[14] ) >> 6  | (data[15] ) << 5) & 0xFF;
        data_tx[22] = ((data[15] ) >> 3) & 0xFF;
        data_tx[23] = data_rx[23];
        data_tx[24] = FOOTER_SBUS;

        for(int tx_index = 0; tx_index < 25;tx_index++){
            sbus_port->write(data_tx[tx_index]);
        }
    }
}