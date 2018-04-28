#include <wiringPi.h>
#include <softPwm.h>

#include "../headers/Steering.h"

Steering::Steering(int p) {
    pin = p;
    pinMode(pin, OUTPUT);
    digitalWrite(pin, LOW);
    softPwmCreate(pin, 15, 100);
}

void Steering::turn_left(int sharpness) {

    steering_values values;

    switch(sharpness) {
        case 1:
            softPwmWrite(pin, values.SHALLOW_LEFT_STEERING_NUM);
            delay(300); 
            break;

        case 2:
            softPwmWrite(pin, values.MEDIUM_LEFT_STEERING_NUM);
            delay(300); 
            break;

        case 3:
            softPwmWrite(pin, values.MAX_LEFT_STEERING_NUM);
            delay(500); 
            break;
    }
}

void Steering::turn_right(int sharpness) {

    steering_values values;

    switch(sharpness) {
        case 1:
            softPwmWrite(pin, values.SHALLOW_RIGHT_STEERING_NUM);
            delay(300); 
            break;

        case 2:
            softPwmWrite(pin, values.MEDIUM_RIGHT_STEERING_NUM);
            delay(300); 
            break;

        case 3:
            softPwmWrite(pin, values.MAX_RIGHT_STEERING_NUM);
            delay(500); 
            break;
    }
}

void Steering::reset_to_forward() {
    steering_values values;

    softPwmWrite(pin, values.FORWARD_STEERING_NUM);
    delay(500);
}