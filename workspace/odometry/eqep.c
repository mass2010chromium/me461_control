/*
 * eqps.c
 *
 *  Created on: Nov 5, 2021
 *      Author: jcpen
 */

#include "eqep.h"

eQEP eQEP1;
eQEP eQEP2;

void init_eQEP(volatile struct EQEP_REGS* regs, eQEP* eqep, uint16_t pin1, uint16_t pin2, uint16_t mux, float factor) {
    setupGPIO(pin1, mux, GPIO_INPUT, GPIO_QUAL6);
    setupGPIO(pin2, mux, GPIO_INPUT, GPIO_QUAL6);
//    GpioCtrlRegs.GPAPUD.bit.GPIO20 = 1; // Disable pull-up on GPIO20 (EQEP1A)
//    GpioCtrlRegs.GPAQSEL2.bit.GPIO20 = 2; // Qual every 6 samples
//    GPIO_SetupPinMux(20, GPIO_MUX_CPU1, 1);

    regs->QEPCTL.bit.QPEN = 0; // make sure eqep in reset
    regs->QDECCTL.bit.QSRC = 0; // Quadrature count mode
    regs->QPOSCTL.all = 0x0; // Disable eQep Position Compare
    regs->QCAPCTL.all = 0x0; // Disable eQep Capture
    regs->QEINT.all = 0x0; // Disable all eQep interrupts
    regs->QPOSMAX = 0xFFFFFFFF; // use full range of the 32 bit count
    regs->QEPCTL.bit.FREE_SOFT = 2; // EQep uneffected by emulation suspend in Code Composer
    regs->QPOSCNT = 0;
    regs->QEPCTL.bit.QPEN = 1; // Enable EQep
    eqep->regs = regs;
    eqep->factor = factor;
    eqep->zero = regs->QPOSCNT;
}


void init_eQEPs(void) {
    // setup eQEP1 pins for input
    init_eQEP(&EQep1Regs, &eQEP1, 20, 21, 1, -COUNT_TO_WHEEL_RAD);// / RAD_PER_FOOT);
    init_eQEP(&EQep2Regs, &eQEP2, 54, 55, 5, COUNT_TO_WHEEL_RAD);// / RAD_PER_FOOT);
}

void zero_eQEP(eQEP* eqep) {
    int32_t raw = eqep->regs->QPOSCNT;
    eqep->zero = raw;
}

float read_eQEP(eQEP* eqep) {
    int32_t raw = eqep->regs->QPOSCNT;
    return (raw - eqep->zero) * eqep->factor;
}

