#include "Motor_PWM_Nano33.h"

Motor_Control::Motor_Control(int forward_pin, int reverse_pin) {
    pwm_forward = new mbed::PwmOut(digitalPinToPinName(forward_pin));
    pwm_reverse = new mbed::PwmOut(digitalPinToPinName(reverse_pin));
}

bool Motor_Control::setFreqAndDuty(int direction, int frequency, float duty_cycle) {
    float period_s = 1.0 / frequency;
    int period_us = period_s * 1e6;

    if (direction == FORWARD) {
        pwm_forward->period_us(period_us);
        pwm_forward->write(duty_cycle);

        pwm_reverse->write(0.0); /* make sure other pin is off */

        return true;
    }
    else if (direction == REVERSE) {
        pwm_reverse->period_us(period_us);
        pwm_reverse->write(duty_cycle);

        pwm_forward->write(0.0); /* make sure other pin is off */
        return true;
    } 
    else {
        return false;
    }
}

bool Motor_Control::setPulseWidth(int direction, int pulsewidth_us) {
    if (direction == FORWARD) {
        pwm_forward->pulsewidth_us(pulsewidth_us);

        //pwm_reverse->pulsewidth_us(0); /* make sure other pin is off */
        pwm_reverse->write(0.0);
        return true;
    }
    else if (direction == REVERSE) {
        pwm_reverse->pulsewidth_us(pulsewidth_us);

        //pwm_forward->pulsewidth_us(0); /* make sure other pin is off */
        pwm_forward->write(0.0);
        return true;
    } 
    else {
        return false;
    }
}

void Motor_Control::pause() {
    pwm_forward->suspend();
    pwm_reverse->suspend();
}

void Motor_Control::resume() {
    pwm_forward->resume();
    pwm_reverse->resume();
}