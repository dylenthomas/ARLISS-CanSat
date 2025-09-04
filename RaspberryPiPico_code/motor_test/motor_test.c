#include <stdio.h>
#include "pico/stdlib.h"
#include "PWMmotorDriver.h"

#define m1_IN1 15
#define m1_IN2 13
#define m2_IN1 6
#define m2_IN2 7
#define min_DC 0.4
#define max_DC 1.0

struct motor Motor1;
struct motor Motor2;

int main()
{
    stdio_init_all();

    addPins(&Motor1, m1_IN1, m1_IN2);
    addPins(&Motor2, m2_IN1, m2_IN2);

    setMotorPWM(Motor1, 1.0, FORWARD);
    setMotorPWM(Motor2, 1.0, FORWARD);
}
