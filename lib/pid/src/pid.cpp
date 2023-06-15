#include "pid.h"

int pid::compute(int current_value, int target_value, double kp, double ki, double kd){
    current_millis = micros();
    if(current_millis - previous_millis > period){
        float time = (current_millis - previous_millis) * 0.000001;
        previous_millis = current_millis;
        prev_error = error;
        error = target_value - current_value;

        proportional_value = kp * error;
        integral_value += ki * error * time;
        integral_value = constrain(integral_value, -1023, 1023);
        differential_value = kd * (error - prev_error) / time;
    }
    return constrain(proportional_value + integral_value + differential_value, 0, 1023);
}

void pid::reset(){
    proportional_value = 0;
    integral_value = 0;
    differential_value = 0;
    current_millis = micros();
    if(current_millis - previous_millis > period){
        previous_millis = current_millis;
    }
}