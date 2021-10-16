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
#include "spi_devices/spi.h"
#include "spi_devices/MPU9250.h"
#include "gpio_decl.h"

#define PI          3.1415926535897932384626433832795
#define TWOPI       6.283185307179586476925286766559
#define HALFPI      1.5707963267948966192313216916398

const int16_t DISPLAY_LEDS[16] = {22, 94, 95, 97, 111, 130, 131, 25, 26, 27, 60, 61, 157, 158, 159, 160};

void play_note(PWM* pwm, float frequency) {
    uint16_t note = ((uint16_t)(((50000000/4)/2)/frequency));
    *(pwm->prd) = note;
}

/**
 * Lowest 5 significant bits of the input are used to set the state of the 5 LED rows.
 */
void SetLEDRowsOnOff(int16_t rows) {

    // ROW 1 Off Use SET.bit for On and TOGGLE.bit fto toggle On/Off
    int16_t i;
    rows = rows & 0x1f;
    for (i = 0; i < 5; ++i) {
        int16_t bit = rows & 0x1;
        rows = rows >> 1;
        setGPIO(DISPLAY_LEDS[i], bit);
        setGPIO(DISPLAY_LEDS[i+5], bit);
        setGPIO(DISPLAY_LEDS[i+10], bit);
    }
}

/**
 * Read state of the four pushbuttons, in bit-packed form.
 */
int16_t ReadSwitches() {
    return GPIO_GET(A, 4) | (GPIO_GET(A, 5)<<1) | (GPIO_GET(A, 6)<<2) | (GPIO_GET(A, 7)<<3);
}

#endif /* COMMON_H_ */
