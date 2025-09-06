#include "PWMmotorDriver.h"

static bool configured_slices[8] = {false, false, false, false, false, false, false, false};

static uint getChannel(uint pwm_pin) {
    if (pwm_pin % 2 == 0) {
        return PWM_CHAN_A;
    }
    else {
        return PWM_CHAN_B;
    }
}

static void configPWMpin(uint pwm_pin) {
    gpio_set_function(pwm_pin, GPIO_FUNC_PWM);
    uint slice_num = pwm_gpio_to_slice_num(pwm_pin);
    
    if (!configured_slices[slice_num]) {
        pwm_set_wrap(slice_num, 99); // 100 cycles
        //pwm_set_clkdiv(slice_num, 125);
        configured_slices[slice_num] = true;
    }
}

void addPins(struct motor* motor_pins, uint IN1, uint IN2) {
    configPWMpin(IN1);
    configPWMpin(IN2);

    motor_pins->IN1 = IN1;
    motor_pins->IN2 = IN2;
}

void setMotorPWM(struct motor motor_pins, float duty_cycle, bool direction) {
    uint drive_pin, low_pin;
    if (direction) {
        drive_pin = motor_pins.IN1;
        low_pin = motor_pins.IN2;
    }
    else {
        drive_pin = motor_pins.IN2;
        low_pin = motor_pins.IN1;
    }
    uint slice_num_drive = pwm_gpio_to_slice_num(drive_pin);
    uint chan_drive = getChannel(drive_pin);
    uint slice_num_low = pwm_gpio_to_slice_num(low_pin);
    uint chan_low = getChannel(low_pin);

    uint16_t wrap_val = pwm_hw->slice[slice_num_drive].top;
    uint16_t flip_val = (uint16_t)(duty_cycle * wrap_val);

    gpio_set_function(drive_pin, GPIO_FUNC_PWM);
    pwm_set_chan_level(slice_num_drive, chan_drive, flip_val);
    pwm_set_enabled(slice_num_drive, true);

    gpio_set_function(low_pin, GPIO_FUNC_SIO);
    gpio_set_dir(low_pin, true);
    gpio_put(low_pin, 0);  
}

void pausePWM() {
    for (int i = 0; i < 8; i++) {
        if (configured_slices[i]) {
            pwm_set_enabled(i, false);
        }
    }
}

void unpausePWM() {
    for (int i = 0; i < 8; i++) {
        if (configured_slices[i]) {
            pwm_set_enabled(i, true);
        }
    }
}
