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

PWM pwm12;
PWM pwm2;
PWM pwm5;
PWM pwm8;
PWM pwm9;

MPU9250 imu;

uint32_t numTimer0calls = 0;
uint32_t numTimer2calls = 0;
uint32_t numSWIcalls = 0;
uint32_t numRXA = 0;
uint16_t UARTPrint = 0;
uint16_t LEDdisplaynum = 0;
int16_t switchStates = 0;

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

/*
 * I am tired so I will re-read this later
 * http://blog.tkjelectronics.dk/2012/09/a-practical-approach-to-kalman-filter-and-how-to-implement-it/
 */
#define CONTROL_TARGET 1.365
float angle_filtered = CONTROL_TARGET;
float angle_raw = CONTROL_TARGET;
float control_target = CONTROL_TARGET;
float Kp = 8;
float bias_est = 0.0f;
float p00 = 0.0f;
float p01 = 0.0f;
float p10 = 0.0f;
float p11 = 0.0f;
float motor_I = 0.0f;

#define dt (1 / 4000.0)
#define Q_ANGLE 0.001
#define Q_BIAS 0.003
#define R_MEASURE 0.3

void __MPU9250_recv(SPI* spi);

void dan_int(SPI* spi) {

    recv[0] = spi_b.regs->SPIRXBUF;
    recv[0] = spi_b.regs->SPIRXBUF;
    recv[0] = spi_b.regs->SPIRXBUF;
    GPIO_SET(A, 16, 1);

    SPI_start(&spi_b, 16, 0, __MPU9250_recv);   // maybe dangerous...

    GPIO_SET(B, 52, 0);
    SPI_ACK(spi);
    MPU9250_set_read_accel_gyro(&imu);
}

// cpu_timer1_isr - CPU Timer1 ISR
__interrupt void cpu_timer1_isr()
{
    CpuTimer1.InterruptCount++;
    if (imu.status == MPU9250_IDLE) {
        MPU9250_update_stats(&imu);

        // lol kalman filter
        float rate = imu.gyro_x - bias_est;
        angle_filtered += rate * dt;
        p00 += dt * (p11 - p01 - p10 + Q_ANGLE);
        p01 -= dt * p11;
        p10 -= dt * p11;
        p11 += dt * Q_BIAS;

        angle_raw = __atan2(imu.accel_y, imu.accel_z);
        float innovation = angle_raw - angle_filtered;
        float S = p00 + R_MEASURE;
        float k0 = p00 / S;
        float k1 = p10 / S;

        angle_filtered += k0 * innovation;
        bias_est += k1 * innovation;

        p11 -= k1 * p01;
        p10 -= k1 * p00;
        p01 -= k0 * p01;
        p00 -= k0 * p00;
        GPIO_SET(B, 52, 1);

        SPI_start(&spi_b, 16, 16, dan_int);   // Setting for DAN28027

        float err = angle_filtered - control_target;
        motor_I -= 0.0002 * err;
//        motor_I *= 0.9995;
        if (motor_I > 1) {
            motor_I = 1;
        }
        else if (motor_I < -1) {
            motor_I = -1;
        }
        float motor_command = motor_I - Kp*err - rate*0.1;
        if (motor_command > 1) {
            motor_command = 1;
        }
        else if (motor_command < -1) {
            motor_command = -1;
        }

        data[0] = 0x00DA;
        data[1] = ((int) (motor_command * 1500)) + 1500;
        data[2] = 1500 - ((int) (motor_command * 1500));

        GPIO_SET(A, 16, 0);
        SPI_write(&spi_b, 9, 3, data, recv);
    }
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

//adcd1 pie interrupt
__interrupt void ADCD_ISR (void) {
    // Print ADCIND0’s voltage value to TeraTerm every 100ms
    AdcdRegs.ADCINTFLGCLR.bit.ADCINT1 = 1; //clear interrupt flag
    PieCtrlRegs.PIEACK.all = PIEACK_GROUP1;
}

#endif

