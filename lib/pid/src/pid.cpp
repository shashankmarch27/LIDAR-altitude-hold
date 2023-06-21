#include "pid.h"

int pid::compute(int current_value, int target_value, double kp, double ki, double kd){
    current_micros = micros();
    if(current_micros - previous_micros > period){
        double time = (current_micros - previous_micros);
        previous_micros = current_micros;
        prev_error = error;
        error = target_value - current_value;

        proportional_value = kp * error;
        integral_value += ki * error * time * 0.000001;
        integral_value = constrain(integral_value, -1023, 1023);
        differential_value = kd * (error - prev_error)*1000000 / time;
    }
    return constrain(proportional_value + integral_value + differential_value, 0, 1023);
}

void pid::reset(int default_value){
    integral_value = default_value;
    proportional_value = 0;
    differential_value = 0;
    current_micros = micros();
    if(current_micros - previous_micros > period){
        previous_micros = current_micros;
    }
}