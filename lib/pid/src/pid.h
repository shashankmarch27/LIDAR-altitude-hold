#ifndef PID_H
#define PID_H

#include <Arduino.h>

class pid{
private:
    unsigned long current_micros;
    unsigned long previous_micros;
    int error;
    int prev_error;
    int period;
    double proportional_value;
    double differential_value;

public:
    double integral_value;

    pid(int frequency){
        period = 1000000 / frequency;
    }

    int compute(int current_value, int target_value, double kp, double ki, double kd);

    void reset();
};

#endif