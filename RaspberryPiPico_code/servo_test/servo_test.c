#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/pwm.h"

uint chan;
uint slice;

void config_servo_pin(uint pin) {
    gpio_set_function(pin, GPIO_FUNC_PWM);
    slice = pwm_gpio_to_slice_num(pin);
    pwm_set_wrap(slice, 20000 - 1);
    pwm_set_clkdiv(slice, 125);
    pwm_set_enabled(slice, true);

    if (pin % 2 == 0) {
        chan = PWM_CHAN_A;
    }
    else {
        chan = PWM_CHAN_B;
    }
}

void set_servo(int deg) {
    int flip_val = (int)(800 + 7.78 * deg);
    pwm_set_chan_level(slice, chan, flip_val);
}

int main()
{
    stdio_init_all();

    config_servo_pin(0);
    sleep_ms(10000);

    for (int i = 0; i < 21; i++) {
        int deg = 10 * i;
        set_servo(180 - deg);
        sleep_ms(500);
    }
}