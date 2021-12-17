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
#include "spi_devices/manual_spi.h"

DAC dacA;
DAC dacB;

#define MOTOR_CONTROL_SCALE = 10.0f;
#define SERVO_CONTROL_SCALE = 90.0f;

MPU9250 imu;

uint32_t numTimer0calls = 0;
uint32_t numTimer2calls = 0;
uint32_t numSWIcalls = 0;
uint32_t numRXA = 0;
uint16_t UARTPrint = 0;
uint16_t LEDdisplaynum = 0;
int16_t switchStates = 0;

volatile RobotCommand commands[2] = {0};
volatile char recv_buf[8];
volatile int16_t recv_state = 0;

// This function is called each time a char is received over UARTA.
void serialRXA(serial_t *s, char data) {
    numRXA ++;
}

void serialRXC(serial_t *s, char data) {
    if (recv_state == -1 && data == '\n') {
        recv_state = 0;
        return;
    }
    if (recv_state == 8) {
        if (data != '\n') {
            recv_state = -1;
            return;
        }
        RobotCommand* current = robot.cmd;
        int16_t index = current - commands;
        int16_t new_index = index ^ 1;
        RobotCommand* fill = &commands[new_index];
        fill->cmd_vel = deserialize_float(recv_buf);
        fill->cmd_omega = deserialize_float(recv_buf + 4);
        fill->age = 0;
        robot.cmd = fill;
        recv_state = 0;
    }
    recv_buf[recv_state] = data;
    ++recv_state;
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

uint16_t led_cmd = 0;
uint16_t data[3] = {0x00DA, 0, 0};
uint16_t recv[3];

// cpu_timer0_isr - CPU Timer0 ISR
__interrupt void cpu_timer0_isr(void)
{
    CpuTimer0.InterruptCount++;

    numTimer0calls++;

    // Blink LaunchPad Red LED
    GpioDataRegs.GPBTOGGLE.bit.GPIO34 = 1;
    //SPI_write(&spi_b, 9, 3, data, recv);

    // Acknowledge this interrupt to receive more interrupts from group 1
    PieCtrlRegs.PIEACK.all = PIEACK_GROUP1;
}

// cpu_timer1_isr - CPU Timer1 ISR
__interrupt void cpu_timer1_isr()
{
    CpuTimer1.InterruptCount++;
    Robot_update(&robot);
    SPI_flush(&spi_b);
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

int16_t adcA0;
int16_t adcA1;
float adcA0F;
float adcA1F;
int32_t adcA_count;
float lowpass_21[22]={   -2.3890045153263611e-03,    -3.3150057635348224e-03,    -4.6136191242627002e-03,    -4.1659855521681268e-03,    1.4477422497795286e-03, 1.5489414225159667e-02, 3.9247886844071371e-02, 7.0723964095458614e-02, 1.0453473887246176e-01, 1.3325672639406205e-01, 1.4978314227429904e-01, 1.4978314227429904e-01, 1.3325672639406205e-01, 1.0453473887246176e-01, 7.0723964095458614e-02, 3.9247886844071371e-02, 1.5489414225159667e-02, 1.4477422497795286e-03, -4.1659855521681268e-03,    -4.6136191242627002e-03,    -3.3150057635348224e-03,    -2.3890045153263611e-03};
sliding_window_init(adcA0_buffer, arraysize(lowpass_21));
sliding_window_init(adcA1_buffer, arraysize(lowpass_21));

__interrupt void ADCA_ISR (void) {
    adcA0 = AdcaResultRegs.ADCRESULT0;
    adcA1 = AdcaResultRegs.ADCRESULT1;
    sliding_window_push(&adcA0_buffer, adcA0 * ADC_TO_VOLT);
    sliding_window_push(&adcA1_buffer, adcA1 * ADC_TO_VOLT);
    adcA0F = filter(lowpass_21, arraysize(lowpass_21), &adcA0_buffer, windowsize(adcA0_buffer));
    adcA1F = filter(lowpass_21, arraysize(lowpass_21), &adcA1_buffer, windowsize(adcA1_buffer));

    ++adcA_count;

    AdcaRegs.ADCINTFLGCLR.bit.ADCINT1 = 1; //clear interrupt flag
    PieCtrlRegs.PIEACK.all = PIEACK_GROUP1;
}

__interrupt void ADCB_ISR (void) {
    AdcbRegs.ADCINTFLGCLR.bit.ADCINT1 = 1; //clear interrupt flag
    PieCtrlRegs.PIEACK.all = PIEACK_GROUP1;
}

//adcd1 pie interrupt
__interrupt void ADCD_ISR (void) {
    // Print ADCIND0’s voltage value to TeraTerm every 100ms
    AdcdRegs.ADCINTFLGCLR.bit.ADCINT1 = 1; //clear interrupt flag
    PieCtrlRegs.PIEACK.all = PIEACK_GROUP1;
}

#endif

