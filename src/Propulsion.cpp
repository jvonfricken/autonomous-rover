#include <wiringPi.h>
#include <softPwm.h>

#include "../headers/Propulsion.h"

Propulsion::Propulsion(int r1Pin, int r2Pin, int tPin) {
    relay1Pin = r1Pin;
    relay2Pin = r2Pin;
    throttlePin = tPin;

    //Prepare GPIO pins
    pinMode(relay1Pin, OUTPUT);
    pinMode(relay2Pin, OUTPUT);
    pinMode(throttlePin, OUTPUT);

    //Set everything off init
    digitalWrite(relay1Pin, LOW);
    digitalWrite(relay2Pin, LOW);
    digitalWrite(throttlePin, LOW);

    //Configure the throttle pin for PWM
    softPwmCreate(throttlePin, 0, 100);
}

void Propulsion::drive(int speed) {
    //Switch polarity forward
    digitalWrite(relay1Pin, LOW);
    digitalWrite(relay2Pin, LOW);

    softPwmWrite(throttlePin, speed);
}

void Propulsion::reverse(int speed) {
    //Reverse polarity
    digitalWrite(relay1Pin, HIGH);
    digitalWrite(relay2Pin, HIGH);

    //Set Throttle
    softPwmWrite(throttlePin, speed);
}

void Propulsion::stop(int direction = 1) {
    if(direction == 1) {
        //Stop
        digitalWrite(relay1Pin, LOW);
        digitalWrite(relay2Pin, LOW);
    } else {
        digitalWrite(relay1Pin, HIGH);
        digitalWrite(relay2Pin, HIGH);
    }

    delay(50);

    //shut off the power
    softPwmWrite(throttlePin, 0);
    digitalWrite(throttlePin, LOW);
}
