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
#include "filter.h"

#define PI          3.1415926535897932384626433832795
#define TWOPI       6.283185307179586476925286766559
#define HALFPI      1.5707963267948966192313216916398


#define GPIOSET(x, n, a) if(a) {GpioDataRegs.GP ## x ## SET.bit.GPIO ## n = 1;} else {GpioDataRegs.GP ## x ## CLEAR.bit.GPIO ## n = 1;}
#define GPIOGET(x, n) (GpioDataRegs.GP ## x ## DAT.bit.GPIO ## n)

#define setupGPIO(x, n, mux, type, flags) \
GPIO_SetupPinMux(n, GPIO_MUX_CPU1, mux); \
GPIO_SetupPinOptions(n, type, flags);

#define initGPIO(x, n, mux, type, flags, init) \
setupGPIO(x, n, mux, type, flags); \
GPIOSET(x, n, init);


void play_note(PWM* pwm, float frequency) {
    uint16_t note = ((uint16_t)(((50000000/4)/2)/frequency));
    *(pwm->prd) = note;
}

/**
 * Lowest 5 significant bits of the input are used to set the state of the 5 LED rows.
 */
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

/**
 * Read state of the four pushbuttons, in bit-packed form.
 */
int16_t ReadSwitches() {
    return GPIOGET(A, 4) | (GPIOGET(A, 5)<<1) | (GPIOGET(A, 6)<<2) | (GPIOGET(A, 7)<<3);
}

#endif /* COMMON_H_ */
