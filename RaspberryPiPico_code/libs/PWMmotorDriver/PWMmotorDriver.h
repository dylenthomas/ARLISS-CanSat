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
};

static uint getChannel(uint pwm_pin);
static void configPWMpin(uint pwm_pin);

void addPins(struct motor* motor_pins, uint IN1, uint IN2);
void setMotorPWM(struct motor motor_pins, float duty_cycle, bool direction);
void pausePWM();
void unpausePWM();

#endif
