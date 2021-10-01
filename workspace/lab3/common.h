/*
 * common.h
 *
 *  Created on: Oct 1, 2021
 *      Author: jcpen
 */

#ifndef COMMON_H_
#define COMMON_H_
#include <stdio.h>
#include <stdlib.h>
#include <stdarg.h>
#include <string.h>
#include <math.h>
#include <limits.h>
#include "F28x_Project.h"

#include "pwm.h"
#include "adc_dac.h"

#define PI          3.1415926535897932384626433832795
#define TWOPI       6.283185307179586476925286766559
#define HALFPI      1.5707963267948966192313216916398

DAC dacA;
DAC dacB;

#define MOTOR_CONTROL_SCALE = 10.0f;
#define SERVO_CONTROL_SCALE = 90.0f;

float pwm2_control;
uint16_t pwm2_dir;

float servo_control;
uint16_t servo_dir;

int16_t music_index;

PWM pwm12;
PWM pwm2;
PWM pwm5;
PWM pwm8;
PWM pwm9;

int16_t adcD0;
int16_t adcD1;
float adcD0F;

uint16_t adcD_count = 0;
uint32_t numTimer0calls = 0;
uint32_t numTimer2calls = 0;
uint32_t numSWIcalls = 0;
uint32_t numRXA = 0;
uint16_t UARTPrint = 0;
uint16_t LEDdisplaynum = 0;
int16_t switchStates = 0;

#define GPIOSET(x, n, a) if(a) {GpioDataRegs.GP ## x ## SET.bit.GPIO ## n = 1;} else {GpioDataRegs.GP ## x ## CLEAR.bit.GPIO ## n = 1;}
#define GPIOGET(x, n) (GpioDataRegs.GP ## x ## DAT.bit.GPIO ## n)

void SetLEDRowsOnOff(int16_t rows) {

    // ROW 1 Off Use SET.bit for On and TOGGLE.bit fto toggle On/Off
    int bit = (rows & 0x10) == 0x10;    // 4th bit is top row, etc etc.
    GPIOSET(A, 22, bit);
    GPIOSET(E, 130, bit);
    GPIOSET(B, 60, bit);

    // ROW 2 Off Use SET.bit for On and TOGGLE.bit fto toggle On/Off
    bit = (rows & 0x8) == 0x8;
    GPIOSET(C, 94, bit);
    GPIOSET(E, 131, bit);
    GPIOSET(B, 61, bit);

    // ROW 3 Off Use SET.bit for On and TOGGLE.bit fto toggle On/Off
    bit = (rows & 0x4) == 0x4;
    GPIOSET(C, 95, bit);
    GPIOSET(A, 25, bit);
    GPIOSET(E, 157, bit);

    // ROW 4 Off Use SET.bit for On and TOGGLE.bit fto toggle On/Off
    bit = (rows & 0x2) == 0x2;
    GPIOSET(D, 97, bit);
    GPIOSET(A, 26, bit);
    GPIOSET(E, 158, bit);

    // ROW 5 Off Use SET.bit for On and TOGGLE.bit fto toggle On/Off
    bit = (rows & 0x1) == 0x1;
    GPIOSET(D, 111, bit);
    GPIOSET(A, 27, bit);
    GPIOSET(E, 159, bit);
}

int16_t ReadSwitches() {
    return GPIOGET(A, 4) | (GPIOGET(A, 5)<<1) | (GPIOGET(A, 6)<<2) | (GPIOGET(A, 7)<<3);
}

#endif /* COMMON_H_ */
