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

/**
 * Set up one channel of the ADC.
 * REQUIRES EALLOW!
 *
 * @param adc_id : [a, b, c, d]
 * @param channel : channel id (0 to 15)
 * @param period : Period (50MHz clock) minus one
 * @param trigger : Object to trigger on
 */
#define setupADCChannel(adc_id, channel, listen, period, trigger) \
Adc ## adc_id ## Regs.ADCSOC ## channel ## CTL.bit.ACQPS = (period);    /* sample window is acqps + 1 SYSCLK cycles = 500ns */\
Adc ## adc_id ## Regs.ADCSOC ## channel ## CTL.bit.CHSEL = listen;     /* By default each one listens to its own channel. */\
Adc ## adc_id ## Regs.ADCSOC ## channel ## CTL.bit.TRIGSEL = (trigger); /* sample window is acqps + 1 SYSCLK cycles = 500ns */

/**
 * Setup an ADC. (ADC A, B, C, D, one of those.)
 * REQUIRES EALLOW!
 *
 * @param adc_id: [a, b, c, d] (no quotes)
 */
#define setupADC(adc_id, size) \
Adc ## adc_id ## Regs.ADCINTSEL1N2.bit.INT1E = 1; /* enable INT1 flag */\
Adc ## adc_id ## Regs.ADCINTFLGCLR.bit.ADCINT1 = 1; /* make sure INT1 flag is cleared */\
Adc ## adc_id ## Regs.ADCINTSEL1N2.bit.INT1SEL = (size); //set to SOC1, the last converted, and it will set INT1 flag ADCD1

/**
 * Set up a DAC.
 * REQUIRES EALLOW!
 *
 * @param dac_id : [a, b] (literal)
 * @param dac_struct : pointer to a struct of type DAC. Binds the `dacvals` pointer.
 */
#define setupDAC(dac_id, dac_struct) \
Dac ## dac_id ## Regs.DACOUTEN.bit.DACOUTEN = 1; /* Enable DAC output */\
Dac ## dac_id ## Regs.DACCTL.bit.LOADMODE = 1;   /* Load on next sysclk */\
Dac ## dac_id ## Regs.DACCTL.bit.DACREFSEL = 1;  /* Use ADC VREF as reference voltage */\
(dac_struct)->dacvals = & Dac ## dac_id ## Regs.DACVALS;

#define ADC_TO_VOLT (3.0/4095.0)
#define VOLT_TO_ADC (4095.0/3.0)

/**
 * Routine for powering up all four ADCs.
 */
void powerup_ADCs();

/**
 * Short routine for writing a voltage to the DAC output (0-3V).
 */
void setDAC(DAC* dac, float voltage);

#endif /* ADC_DAC_H_ */
