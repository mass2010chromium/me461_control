/*
 * adc_dac.h
 *
 *  Created on: Oct 1, 2021
 *      Author: jcpen
 */

#ifndef ADC_DAC_H_
#define ADC_DAC_H_

#include <stdio.h>
#include <stdlib.h>
#include <stdarg.h>
#include <string.h>
#include <math.h>
#include <limits.h>
#include "F28x_Project.h"

typedef struct {
    volatile union DACVALS_REG* dacvals;
} DAC;

#define setupADCChannel(adc_id, channel, period, trigger) \
Adc ## adc_id ## Regs.ADCSOC ## channel ## CTL.bit.CHSEL = channel;     /* Why is it pin 0 for ADCIN0? */\
Adc ## adc_id ## Regs.ADCSOC ## channel ## CTL.bit.ACQPS = (period);    /* sample window is acqps + 1 SYSCLK cycles = 500ns */\
Adc ## adc_id ## Regs.ADCSOC ## channel ## CTL.bit.TRIGSEL = (trigger); /* sample window is acqps + 1 SYSCLK cycles = 500ns */

#define setupADC(adc_id, period, trigger) \
setupADCChannel(adc_id, 0, period, trigger);\
setupADCChannel(adc_id, 1, period, trigger);\
setupADCChannel(adc_id, 2, period, trigger);\
setupADCChannel(adc_id, 3, period, trigger);\
Adc ## adc_id ## Regs.ADCINTSEL1N2.bit.INT1SEL = 0x3; /* set to SOC3, the last converted, and it will set INT1 flag ADCD1 */\
Adc ## adc_id ## Regs.ADCINTSEL1N2.bit.INT1E = 1; /* enable INT1 flag */\
Adc ## adc_id ## Regs.ADCINTFLGCLR.bit.ADCINT1 = 1; /* make sure INT1 flag is cleared */

#define setupDAC(dac_id, dac_struct) \
Dac ## dac_id ## Regs.DACOUTEN.bit.DACOUTEN = 1; /* Enable DAC output */\
Dac ## dac_id ## Regs.DACCTL.bit.LOADMODE = 1;   /* Load on next sysclk */\
Dac ## dac_id ## Regs.DACCTL.bit.DACREFSEL = 1;  /* Use ADC VREF as reference voltage */\
(dac_struct)->dacvals = & Dac ## dac_id ## Regs.DACVALS;

void setDAC(DAC* dac, float voltage);

#endif /* ADC_DAC_H_ */
