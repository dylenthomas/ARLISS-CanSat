#include "PWMmotorDriver.h"
#include <hardware/pwm.h>
#include <pico/stdlib.h>

static bool configured_slices[8] = [false, false, false, false, false, false, false, false];

static bool sliceConfigured(uint slice_num) {
    if (configured_slices[slice_num]) {
        return configured_slices[slice_num];
    }
    else {
        configured_slices[slice_num] = true;
        return false;
    }

}

static uint getChannel(uint pwm_pin) {
    if (pwm_pin % 2 == 0) {
        return PWM_CHAN_A;
    }
    else {
        return PWM_CHAN_B;
    }
}

void configPWMpin(uint pwm_pin) {
    gpio_set_function(pwm_pin, GPIO_FUNC_PWM);
    uint slice_num = pwm_gpio_to_slice_num(pwm_pin);
    
    if (!sliceConfigured(slice_num)) {
        pwm_set_wrap(slice_num, 99); // 100 cycles
    }
}

void configDirPin(uint direction_pin) {
    gpio_init(direction_pin);
    gpio_set_dir(direction_pin, true);
}

void setPWM_DC(uint pwm_pin, float duty_cycle) {
    uint16_t flip_val = (uint16_t)(duty_cycle * 100);
    uint slice_num = pwm_gpio_to_slice_num(pwm_pin);
    pwm_set_chan_level(slice_num, getChannel(pwm_pin), flip_val);
    pwm_set_enabled(slice_num, true);
}

void setMotor_dir(uint direction_pin, bool direction) {
    gpio_put(direction_pin, direction); 
}

void pausePWM(uint* pwm_pins) {
    int len = sizeof(pwm_pins) / sizeof(pwm_pins[0]);
    for (int i = 0; i < len; i++) {
        uint pwm_pin = pwm_pins[i];
        uint slice_num = pwm_gpio_to_slice_num(pwm_pin);
        pwm_set_enabled(slice_name, false);
    } 
}

void unpausePWM(uint* pwm_pins) {
    int len = sizeof(pwm_pins) / sizeof(pwm_pins[0]);
    for (int i = 0; i < len; i++) {
        uint pwm_pin = pwm_pins[i];
        uint slice_num = pwm_gpio_to_slice_num(pwm_pin);
        pwm_set_enabled(slice_name, true);
    }
}
