/*
 * interrupt_handlers.h
 *
 * A place to put all the things that might be considered interrupt handlers.
 *
 * TL;DR:
 *      Timer 0 handles I2C_command
 *      Timer 2 handles UARTPrint
 *      serialRXA handles interactive terminal
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

#include "common.h"
//#include "spi_devices/manual_spi.h"

uint32_t numTimer0calls = 0;
uint32_t numTimer2calls = 0;
uint32_t numSWIcalls = 0;
uint32_t numRXA = 0;
volatile uint16_t UARTPrint = 0;    // Flag for printing status
volatile uint16_t I2C_command = 0;  // Flag for sending I2C commands

// Lol should be volatile but w/e
// "D mm/dd/yy hh:mm:ss = "
char bs[100] = {'\b','\b','\b','\b','\b','\b','\b','\b','\b','\b','\b','\b','\b','\b','\b','\b','\b','\b','\b','\b','\b','\b'};
char buf[100] = {'D', ' ', 'm', 'm', '/', 'd', 'd', '/', 'y', 'y', ' ', 'h', 'h', ':', 'm', 'm', ':', 's', 's', ' ', '=', ' '};
#define HEAD_ZERO 22
#define HEAD_NUM 19
char* parse = buf+HEAD_ZERO;            // Convenience pointer into the editable section of the string.
volatile uint16_t head = HEAD_ZERO;     // Index into the buffer representing the end of the editable string.
volatile uint16_t serial_change = 0;    // Flag set whenever the buffer changes, to trigger a repaint
volatile uint16_t date_parse = 0;       // Flag for date parsing

// This function is called each time a char is received over UARTA.
void serialRXA(serial_t *s, char data) {
    /**
     * Emulate an interactive terminal.
     *
     * Typed characters get saved in a buffer. There is also another buffer of backspace chars
     * maintained in parallel to erase this line when things get printed.
     * When the buffer is "full" (a complete date) a flag is set to signal that
     * date parsing can happen.
     */
    numRXA++;
    if (data == '\b') {
        if (head > HEAD_ZERO) {
            bs[head] = 0;
            buf[head] = 0;
            --head;
            serial_change = 1;
        }
    }
    else {
        buf[head] = data;
        bs[head] = '\b';
        ++head;
        if (head == HEAD_ZERO + HEAD_NUM) {
            date_parse = 1;
        }
        buf[head] = 0;
        bs[head] = 0;
        serial_change = 1;
    }
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
    I2C_command = 1;

    // Blink LaunchPad Red LED
    GpioDataRegs.GPBTOGGLE.bit.GPIO34 = 1;

    // Acknowledge this interrupt to receive more interrupts from group 1
    PieCtrlRegs.PIEACK.all = PIEACK_GROUP1;
}

// cpu_timer1_isr - CPU Timer1 ISR
__interrupt void cpu_timer1_isr(void)
{
    CpuTimer1.InterruptCount++;
    PieCtrlRegs.PIEACK.all = PIEACK_GROUP1;
}

// cpu_timer2_isr CPU Timer2 ISR
__interrupt void cpu_timer2_isr(void)
{
    // Blink LaunchPad Blue LED
    GpioDataRegs.GPATOGGLE.bit.GPIO31 = 1;
    ++numTimer2calls;
    CpuTimer2.InterruptCount++;
    UARTPrint = 1;
    PieCtrlRegs.PIEACK.all = PIEACK_GROUP1;
}

__interrupt void ADCA_ISR (void) {
    AdcaRegs.ADCINTFLGCLR.bit.ADCINT1 = 1; //clear interrupt flag
    PieCtrlRegs.PIEACK.all = PIEACK_GROUP1;
}

__interrupt void ADCB_ISR (void) {
    AdcbRegs.ADCINTFLGCLR.bit.ADCINT1 = 1; //clear interrupt flag
    PieCtrlRegs.PIEACK.all = PIEACK_GROUP1;
}

__interrupt void ADCD_ISR (void) {
    AdcdRegs.ADCINTFLGCLR.bit.ADCINT1 = 1; //clear interrupt flag
    PieCtrlRegs.PIEACK.all = PIEACK_GROUP1;
}

#endif

