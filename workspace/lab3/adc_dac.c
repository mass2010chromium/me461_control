/*
 * adc_dac.c
 *
 *  Created on: Oct 1, 2021
 *      Author: jcpen
 */

#include "adc_dac.h"

void setDAC(DAC* dac, float voltage) {
    const float scaling = 4095.0/3.0;
    if (voltage > 3.0) voltage = 3.0;
    else if (voltage < 0.0) voltage = 0.0;
   dac->dacvals->bit.DACVALS = voltage * scaling;
}
