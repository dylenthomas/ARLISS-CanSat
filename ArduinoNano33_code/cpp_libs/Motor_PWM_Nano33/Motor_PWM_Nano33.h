/*
Arduino Library to interact with the mbed OS API for PWM and a motor driver similar to the DRV7881 breakout board from Adafruit.

This board is bi direrctional so both passed pins for an individual motor must be PWM capable pins.
*/

#if defined(ARDUINO)
    #include "Arduino.h"
    #include "mbed.h"
#else
    using namespace std;
#endif

#define FORWARD 1
#define REVERSE 0

class Motor_Control {
    public:
        Motor_Control(int forward_pin, int reverse_pin);

        bool setFreqAndDuty(int direction, int frequency, float duty_cycle);
        bool setPulseWidth(int direction, int pulsewidth_us);
        void pause();
        void resume();

    private:
        mbed::PwmOut* pwm_forward;
        mbed::PwmOut* pwm_reverse;
};