/*
 * eqps.h
 *
 *  Created on: Nov 5, 2021
 *      Author: jcpen
 */

#ifndef EQEP_H_
#define EQEP_H_

#include <math.h>
#include "F28x_Project.h"
#include "gpio_decl.h"

// 5 North South magnet poles in the encoder disk so 5 square waves per one revolution of the
// DC motor's back shaft. Then Quadrature Decoder mode multiplies this by 4 so 20 counts per one rev
// of the DC motor's back shaft. Then the gear motor's gear ratio is 30:1.
#define COUNT_TO_WHEEL_RAD (2*M_PI / (4 * 5 * 30))
#define RAD_PER_FOOT (1.0 / 0.19583)

typedef struct {
    float factor;
    int32_t zero;
    volatile struct EQEP_REGS* regs;
} eQEP;

extern eQEP eQEP1;
extern eQEP eQEP2;

void init_eQEP(volatile struct EQEP_REGS* regs, eQEP* eqep, uint16_t pin1, uint16_t pin2, uint16_t mux, float factor);

/**
 * Convenience for lab 6.
 */
void init_eQEPs(void);

void zero_eQEP(eQEP* eqep);

float read_eQEP(eQEP* eqep);

#endif /* EQEP_H_ */
