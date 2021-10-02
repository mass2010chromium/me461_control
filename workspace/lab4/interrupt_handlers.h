/*
 * interrupt_handlers.h
 *
 *  Created on: Oct 1, 2021
 *      Author: jcpen
 */
#ifndef INTERRUPT_HANDLERS_H_
#define INTERRUPT_HANDLERS_H_
#include <stdio.h>
#include <stdlib.h>
#include <stdarg.h>
#include <string.h>
#include <math.h>
#include <limits.h>
#include "F28x_Project.h"
#include "driverlib.h"
#include "LEDPatterns.h"
#include "F2837xD/f28379dSerial.h"

#include "pwm.h"
#include "common.h"

DAC dacA;
DAC dacB;

#define MOTOR_CONTROL_SCALE = 10.0f;
#define SERVO_CONTROL_SCALE = 90.0f;

PWM pwm12;
PWM pwm2;
PWM pwm5;
PWM pwm8;
PWM pwm9;

int16_t adcD0;
int16_t adcD1;
float adcD0F;

int16_t adcA0;
int16_t adcA1;

uint32_t adcD_count = 0;
uint32_t numTimer0calls = 0;
uint32_t numTimer2calls = 0;
uint32_t numSWIcalls = 0;
uint32_t numRXA = 0;
uint16_t UARTPrint = 0;
uint16_t LEDdisplaynum = 0;
int16_t switchStates = 0;

sliding_window_init(adcD_buffer, 10);

// This function is called each time a char is received over UARTA.
void serialRXA(serial_t *s, char data) {
    numRXA ++;
}

// SWI_isr,  Using this interrupt as a Software started interrupt
__interrupt void SWI_isr(void) {

    // These three lines of code allow SWI_isr, to be interrupted by other interrupt functions
    // making it lower priority than all other Hardware interrupts.
    PieCtrlRegs.PIEACK.all = PIEACK_GROUP12;
    asm("       NOP");                    // Wait one cycle
    EINT;                                 // Clear INTM to enable interrupts

    // Insert SWI ISR Code here.......

    numSWIcalls++;

    DINT;

}

// cpu_timer0_isr - CPU Timer0 ISR
__interrupt void cpu_timer0_isr(void)
{
    CpuTimer0.InterruptCount++;

    numTimer0calls++;

//    if ((numTimer0calls%50) == 0) {
//        PieCtrlRegs.PIEIFR12.bit.INTx9 = 1;  // Manually cause the interrupt for the SWI
//    }

    if ((numTimer0calls%250) == 0) {
        displayLEDletter(LEDdisplaynum);
        LEDdisplaynum++;
        if (LEDdisplaynum == 0xFFFF) {  // prevent roll over exception
            LEDdisplaynum = 0;
        }
    }

    // Blink LaunchPad Red LED
    GpioDataRegs.GPBTOGGLE.bit.GPIO34 = 1;

    // Acknowledge this interrupt to receive more interrupts from group 1
    PieCtrlRegs.PIEACK.all = PIEACK_GROUP1;
}

// cpu_timer1_isr - CPU Timer1 ISR
__interrupt void cpu_timer1_isr(void)
{
    CpuTimer1.InterruptCount++;
}

// cpu_timer2_isr CPU Timer2 ISR
__interrupt void cpu_timer2_isr(void)
{
    // Blink LaunchPad Blue LED
    GpioDataRegs.GPATOGGLE.bit.GPIO31 = 1;
    ++numTimer2calls;
    CpuTimer2.InterruptCount++;
    UARTPrint = 1;
}

__interrupt void ADCA_ISR (void) {
    adcA0 = AdcaResultRegs.ADCRESULT0;
    adcA1 = AdcaResultRegs.ADCRESULT1;

    AdcaRegs.ADCINTFLGCLR.bit.ADCINT1 = 1; //clear interrupt flag
    PieCtrlRegs.PIEACK.all = PIEACK_GROUP1;
}

//adcd1 pie interrupt
__interrupt void ADCD_ISR (void) {
    adcD0 = AdcdResultRegs.ADCRESULT0;
    adcD1 = AdcdResultRegs.ADCRESULT1;

    adcD0 = (adcD_count*16) % 4096;

    ++adcD_count;
    // Here covert ADCIND0 to volts
    adcD0F = adcD0 * (3.0 / 4095.0);

    sliding_window_push(adcD_buffer, adcD0F);

    // Here write voltages value to DACA
    setDAC(&dacA, 3 - adcD0F);

    // Print ADCIND0’s voltage value to TeraTerm every 100ms
    AdcdRegs.ADCINTFLGCLR.bit.ADCINT1 = 1; //clear interrupt flag
    PieCtrlRegs.PIEACK.all = PIEACK_GROUP1;
}

#endif

