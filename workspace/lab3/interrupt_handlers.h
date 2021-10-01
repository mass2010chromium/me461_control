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
#include "song.h"

#include "common.h"

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
    if (music_index < SONG_LENGTH*4) {
        if (music_index & 1) {
            (*pwm9.prd) = songarray[(music_index/2) | 1];
        }
        else {
            (*pwm9.prd) = songarray[(music_index/2) & ~1];
        }
        ++music_index;
    }

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

    if (pwm2_control > pwm2.control_max) {
        pwm2_dir = 1;
    }
    if (pwm2_control < pwm2.control_min) {
        pwm2_dir = 0;
    }
    if (pwm2_dir) {
        pwm2_control -= 0.002;
    }
    else {
        pwm2_control += 0.002;
    }

    if (servo_control > pwm8.control_max) {
        servo_dir = 1;
    }
    if (servo_control < pwm8.control_min) {
        servo_dir = 0;
    }
    if (servo_dir) {
        servo_control -= 0.05;
    }
    else {
        servo_control += 0.05;
    }
}

//adcd1 pie interrupt
__interrupt void ADCD_ISR (void) {
    adcD0 = AdcdResultRegs.ADCRESULT0;
    adcD1 = AdcdResultRegs.ADCRESULT1;

    ++adcD_count;
    adcD0F = adcD0 * (3.0 / 4095.0);

    // Here covert ADCIND0 to volts
    // Here write voltages value to DACA
    // Print ADCIND0’s voltage value to TeraTerm every 100ms
    AdcdRegs.ADCINTFLGCLR.bit.ADCINT1 = 1; //clear interrupt flag
    PieCtrlRegs.PIEACK.all = PIEACK_GROUP1;
}

#endif

