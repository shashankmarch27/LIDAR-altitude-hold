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
    float proportional_value;
    float integral_value;
    float differential_value;

public:

    int compute(int current_value, int target_value, float kp, float ki, float kd);
};

#endif