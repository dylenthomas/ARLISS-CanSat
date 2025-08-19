#ifndef PWMMOTORDRIVER_H

#define PWMMOTORDRIVER_H

#define FORWARD true
#define REVERSE false 

static bool sliceConfigured(uint slice_num);
static uint getChannel(uint pwm_pin);

void configPWMpin(uint pwm_pin);
void configDirPin(uint direction_pin);

void setPWM_DC(uint pwm_pin, float duty_cycle);
void setMotor_dir(uint direction_pin, bool direction);

void pausePWM(uint* pwm_pins);
void unpausePWM(uint* pwm_pins);

#endif
