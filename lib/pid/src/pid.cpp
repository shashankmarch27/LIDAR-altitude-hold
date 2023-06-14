#include "pid.h"

int pid::compute(int current_value, int target_value, float kp, float ki, float kd){
    current_millis = micros();
    if(current_millis - previous_millis > 500){
        float time = (current_millis - previous_millis) * 0.000001;
        previous_millis = current_millis;
        prev_error = error;
        error = target_value - current_value;

        proportional_value = kp * error;
        integral_value += ki * error * time;
        integral_value = constrain(integral_value, 0, 1023);
        differential_value = kd * (error - prev_error) * time;
    }

    return constrain(proportional_value + integral_value + differential_value, 0, 1023);
}

void pid::reset(){
    integral_value = 0;
    differential_value = 0;
}