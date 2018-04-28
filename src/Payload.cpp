#include <wiringPi.h>
#include <softPwm.h>

#include "../headers/Payload.h"

void Payload::set_back_position() {
    softPwmWrite(pin, END_NUM);
    delay(500);
}

void Payload::set_forward_position() {
    softPwmWrite(pin, START_NUM);
    delay(500);
}

Payload::Payload(int p) {
    pin = p;
    pinMode(pin, OUTPUT);
    digitalWrite(pin, LOW);
    softPwmCreate(pin, START_NUM, PWM_RANGE);
}

void Payload::drop_payload() {
    set_back_position();
    set_forward_position();
}
