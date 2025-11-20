#ifndef PWMMOTORDRIVER_H

#define PWMMOTORDRIVER_H
#include <hardware/pwm.h>
#include <pico/stdlib.h>
#include <stdbool.h>

#define FORWARD true
#define REVERSE false

struct motor {
    int IN1;
    int IN2;
    uint slice_IN1, slice_IN2;
    uint chan_IN1, chan_IN2;
    int wrap;
};

void addPins(struct motor* motor_pins, uint IN1, uint IN2);
void setMotorPWM(struct motor* motor_pins, float duty_cycle, bool forward);
void pausePWM();
void unpausePWM();

#endif
