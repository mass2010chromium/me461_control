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

uint32_t adcD_count = 0;
uint32_t numTimer0calls = 0;
uint32_t numTimer2calls = 0;
uint32_t numSWIcalls = 0;
uint32_t numRXA = 0;
uint16_t UARTPrint = 0;
uint16_t LEDdisplaynum = 0;
int16_t switchStates = 0;

float* active_filter;
float filter_size;
float average_filter[] = {0.2, 0.2, 0.2, 0.2, 0.2};
float lowpass_4[5] = {3.3833240118424500e-02, 2.4012702387971543e-01, 4.5207947200372001e-01, 2.4012702387971543e-01, 3.3833240118424500e-02};
float lowpass_21[22]={   -2.3890045153263611e-03,    -3.3150057635348224e-03,    -4.6136191242627002e-03,    -4.1659855521681268e-03,    1.4477422497795286e-03, 1.5489414225159667e-02, 3.9247886844071371e-02, 7.0723964095458614e-02, 1.0453473887246176e-01, 1.3325672639406205e-01, 1.4978314227429904e-01, 1.4978314227429904e-01, 1.3325672639406205e-01, 1.0453473887246176e-01, 7.0723964095458614e-02, 3.9247886844071371e-02, 1.5489414225159667e-02, 1.4477422497795286e-03, -4.1659855521681268e-03,    -4.6136191242627002e-03,    -3.3150057635348224e-03,    -2.3890045153263611e-03};
float lowpass_31[32]={   -3.1938239761966587e-04,    -1.0866864938210157e-03,    -2.2911384606021153e-03,    -4.0429413570708655e-03,    -6.0345429962702663e-03,    -7.4517327823236017e-03,    -7.0532560736233556e-03,    -3.4241174469018575e-03,    4.6476146297203492e-03, 1.7765721575552403e-02, 3.5607761202951782e-02, 5.6796595065922689e-02, 7.9020296791706066e-02, 9.9395375517550705e-02, 1.1500184311693601e-01, 1.2346859010789270e-01, 1.2346859010789270e-01, 1.1500184311693601e-01, 9.9395375517550705e-02, 7.9020296791706066e-02, 5.6796595065922689e-02, 3.5607761202951782e-02, 1.7765721575552403e-02, 4.6476146297203492e-03, -3.4241174469018575e-03,    -7.0532560736233556e-03,    -7.4517327823236017e-03,    -6.0345429962702663e-03,    -4.0429413570708655e-03,    -2.2911384606021153e-03,    -1.0866864938210157e-03,    -3.1938239761966587e-04};
float bandpass_100[101]={  2.0768470653189774e-04, 8.9713855892880766e-05, -3.9332320027400035e-04,    -5.5390765443489773e-04,    2.5618634242491491e-04, 1.1218014896390519e-03, 4.3145705167366789e-04, -1.3258118112723196e-03,    -1.5262320596691505e-03,    6.2020738295731018e-04, 2.1706732201904558e-03, 6.8309098550924617e-04, -1.6845361592173263e-03,    -1.4782296115862432e-03,    4.2133352634961903e-04, 7.3378082086456345e-04, -4.3533287584822219e-05,    9.4502154172264774e-04, 1.9846177665158076e-03, -1.1825837306078460e-03,    -5.4730691588955717e-03,    -2.2372879178065158e-03,    7.1267889940326481e-03, 8.4841478564812740e-03, -3.6494259398237193e-03,    -1.3125893698374066e-02,    -4.3459058236371409e-03,    1.1519131325551881e-02, 1.1325530327988491e-02, -3.9820620003508797e-03,    -1.1147295728596501e-02,    -2.6182816424636527e-03,    3.8594794688248217e-03, 1.0707079124863483e-04, 1.6757372834788288e-03, 1.2022818390179584e-02, 6.0421873372859302e-03, -2.2325574236986223e-02,    -2.9514700272676782e-02,    1.4072421866032667e-02, 5.5268747920345293e-02, 2.0109522308388254e-02, -6.0075528718230367e-02,    -6.7457151066860108e-02,    2.8288203547313682e-02, 9.9264347215343704e-02, 3.2718694804742231e-02, -8.9759223070526806e-02,    -9.2908728032843585e-02,    3.6197920636818107e-02, 1.1796991216750140e-01, 3.6197920636818107e-02, -9.2908728032843585e-02,    -8.9759223070526806e-02,    3.2718694804742231e-02, 9.9264347215343704e-02, 2.8288203547313682e-02, -6.7457151066860108e-02,    -6.0075528718230367e-02,    2.0109522308388254e-02, 5.5268747920345293e-02, 1.4072421866032667e-02, -2.9514700272676782e-02,    -2.2325574236986223e-02,    6.0421873372859302e-03, 1.2022818390179584e-02, 1.6757372834788288e-03, 1.0707079124863483e-04, 3.8594794688248217e-03, -2.6182816424636527e-03,    -1.1147295728596501e-02,    -3.9820620003508797e-03,    1.1325530327988491e-02, 1.1519131325551881e-02, -4.3459058236371409e-03,    -1.3125893698374066e-02,    -3.6494259398237193e-03,    8.4841478564812740e-03, 7.1267889940326481e-03, -2.2372879178065158e-03,    -5.4730691588955717e-03,    -1.1825837306078460e-03,    1.9846177665158076e-03, 9.4502154172264774e-04, -4.3533287584822219e-05,    7.3378082086456345e-04, 4.2133352634961903e-04, -1.4782296115862432e-03,    -1.6845361592173263e-03,    6.8309098550924617e-04, 2.1706732201904558e-03, 6.2020738295731018e-04, -1.5262320596691505e-03,    -1.3258118112723196e-03,    4.3145705167366789e-04, 1.1218014896390519e-03, 2.5618634242491491e-04, -5.5390765443489773e-04,    -3.9332320027400035e-04,    8.9713855892880766e-05, 2.0768470653189774e-04};

#define microphone_filt bandpass_100

sliding_window_init(adcD_buffer, 22);
sliding_window_init(adcA0_buffer, arraysize(lowpass_21));
sliding_window_init(adcA1_buffer, arraysize(lowpass_21));
sliding_window_init(adcB_buffer, arraysize(microphone_filt));
/*
 * int type = 0;
 * ..
 * rxa:
 * switch
 * case '0':
 *      type = 0
 * case '1':
 *      type = 1
 *
 * ..
 * if (type == 0) {
 *    set yk
 * }
 * if type == 1
 *    set yk21
 */

// This function is called each time a char is received over UARTA.
void serialRXA(serial_t *s, char data) {
    numRXA ++;
    switch(data) {
    case '1':
        active_filter = average_filter;
        filter_size = 5;
        break;
    case '2':
        active_filter = lowpass_4;
        filter_size = 5;
        break;
    case '3':
        active_filter = lowpass_21;
        filter_size = 22;
        break;
    case '4':
        play_note(&pwm9, 440.0);
        break;
    case '5':
        play_note(&pwm9, 1000.0);
        break;
    case '6':
        *(pwm9.prd) = 1;
        break;
    default:
        break;
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


int16_t adcA0;
int16_t adcA1;
float adcA0F;
float adcA1F;
int32_t adcA_count;

__interrupt void ADCA_ISR (void) {
    adcA0 = AdcaResultRegs.ADCRESULT0;
    adcA1 = AdcaResultRegs.ADCRESULT1;
    sliding_window_push(adcA0_buffer, adcA0 * ADC_TO_VOLT);
    sliding_window_push(adcA1_buffer, adcA1 * ADC_TO_VOLT);
    adcA0F = filter(lowpass_21, arraysize(lowpass_21), &adcA0_buffer, windowsize(adcA0_buffer));
    adcA1F = filter(lowpass_21, arraysize(lowpass_21), &adcA1_buffer, windowsize(adcA1_buffer));

    ++adcA_count;

    AdcaRegs.ADCINTFLGCLR.bit.ADCINT1 = 1; //clear interrupt flag
    PieCtrlRegs.PIEACK.all = PIEACK_GROUP1;
}

int16_t adcB0;
float adcB0F;
int32_t adcB_count;

__interrupt void ADCB_ISR (void) {
    adcB0 = AdcbResultRegs.ADCRESULT0;
    sliding_window_push(adcB_buffer, adcB0 * ADC_TO_VOLT);

    ++adcB_count;

    setDAC(&dacA, 1.5 + filter(microphone_filt, arraysize(microphone_filt), &adcB_buffer, windowsize(adcB_buffer)));

    AdcbRegs.ADCINTFLGCLR.bit.ADCINT1 = 1; //clear interrupt flag
    PieCtrlRegs.PIEACK.all = PIEACK_GROUP1;
}

int16_t adcD0;
int16_t adcD1;
float adcD0F;

//adcd1 pie interrupt
__interrupt void ADCD_ISR (void) {
    adcD0 = AdcdResultRegs.ADCRESULT0;
    adcD1 = AdcdResultRegs.ADCRESULT1;

    // Spoofed ADC values for filter testing
    // adcD0 = (adcD_count*16) % 4096;

    ++adcD_count;
    // Here covert ADCIND0 to volts
    adcD0F = adcD0 * ADC_TO_VOLT;

    sliding_window_push(adcD_buffer, adcD0F);

    // Here write voltages value to DACA
    setDAC(&dacA, filter(active_filter, filter_size, &adcD_buffer, windowsize(adcD_buffer)));

    // Print ADCIND0’s voltage value to TeraTerm every 100ms
    AdcdRegs.ADCINTFLGCLR.bit.ADCINT1 = 1; //clear interrupt flag
    PieCtrlRegs.PIEACK.all = PIEACK_GROUP1;
}

#endif

