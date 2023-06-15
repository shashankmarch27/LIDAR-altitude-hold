#ifndef PID_H
#define PID_H

#include <Arduino.h>

class pid{
private:
    int current_millis;
    int previous_millis;
    int error;
    int prev_error;
    int period;
    double proportional_value;
    double integral_value;
    double differential_value;

public:
    pid(int frequency){
        period = 1000000 / frequency;
    }

    int compute(int current_value, int target_value, double kp, double ki, double kd);

    void reset();
};

#endif