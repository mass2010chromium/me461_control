//#############################################################################
// FILE:   labstarter_main.c
//
// TITLE:  Lab Starter
//#############################################################################

// Included Files
#include <stdio.h>
#include <stdlib.h>
#include <stdarg.h>
#include <string.h>
#include <math.h>
#include <limits.h>
#include "F28x_Project.h"
#include "driverlib.h"
#include "device.h"
#include "dsp.h"
#include "fpu32/fpu_rfft.h"

#include "interrupt_handlers.h"

void main(void) {
    // PLL, WatchDog, enable Peripheral Clocks
    // This example function is found in the F2837xD_SysCtrl.c file.
    InitSysCtrl();
    InitGpio();
    powerup_ADCs();
	
    initGPIO(A, 31, 0, GPIO_OUTPUT, GPIO_PUSHPULL, 0); // Blue LED on LaunchPad
    initGPIO(B, 34, 0, GPIO_OUTPUT, GPIO_PUSHPULL, 0); // Red LED on LaunchPad

	// LED1 and PWM Pin
    initGPIO(A, 22, 0, GPIO_OUTPUT, GPIO_PUSHPULL, 0); // Red LED on LaunchPad

    initGPIO(A, 16, 5, GPIO_OUTPUT, GPIO_PUSHPULL, 0);  // Buzzer

    EALLOW;
    setupPWM(5, &pwm5, 50000); // 1ms clock
    pwm5.tbctl->bit.CTRMODE = 3;
    pwm5.etsel->bit.SOCAEN = 0;         // Disable Start of Conversion on A group
    pwm5.etsel->bit.SOCASEL = 0b010;    // Fire SoCA when counter == prd
    pwm5.etps->bit.SOCAPRD = 0b01;      // Generate pulse on first event
    pwm5.etsel->bit.SOCAEN = 1;         // Enable SoC A
    *(pwm5.tbctr) = 0;
    pwm5.tbctl->bit.CTRMODE = 0;

    setupADCChannel(d, 0, 0, 99, 0x0D);
    setupADCChannel(d, 1, 1, 99, 0x0D);
    setupADC(d, 1);  // 0x0D: trigger on PWM5 SoCA

    setupPWM(8, &pwm8, 5000); // 0.1ms clock
    pwm8.tbctl->bit.CTRMODE = 3;
    pwm8.etsel->bit.SOCAEN = 0;         // Disable Start of Conversion on A group
    pwm8.etsel->bit.SOCASEL = 0b010;    // Fire SoCA when counter == prd
    pwm8.etps->bit.SOCAPRD = 0b01;      // Generate pulse on first event
    pwm8.etsel->bit.SOCAEN = 1;         // Enable SoC A
    *(pwm8.tbctr) = 0;
    pwm8.tbctl->bit.CTRMODE = 0;

    setupADCChannel(b, 0, 4, 99, 0x13);
    setupADC(b, 0);  // 0x0D: trigger on PWM8 SoCA

    setupADCChannel(a, 0, 2, 99, 0x0D);
    setupADCChannel(a, 1, 3, 99, 0x0D);
    setupADC(a, 1);  // 0x0D: trigger on PWM5 SoCA

    setupDAC(a, &dacA);
    setupDAC(b, &dacB);

    setupPWM(9, &pwm9, ((uint16_t)(((50000000/4)/2)/440.00)));
    pwm9.tbctl->bit.CLKDIV = 0b010;     // Divide by 4
    pwm9.aqctla->bit.CAU = 0b00;        // Disable CMPA
    pwm9.aqctla->bit.ZRO = 0b11;        // Toggle on zero
    EDIS;

    initGPIO(C, 94, 0, GPIO_OUTPUT, GPIO_PUSHPULL, 0);  // LED2
    initGPIO(C, 95, 0, GPIO_OUTPUT, GPIO_PUSHPULL, 0);  // LED3
    initGPIO(D, 97, 0, GPIO_OUTPUT, GPIO_PUSHPULL, 0);  // LED4
    initGPIO(D, 111, 0, GPIO_OUTPUT, GPIO_PUSHPULL, 0); // LED5
    initGPIO(E, 130, 0, GPIO_OUTPUT, GPIO_PUSHPULL, 0); // LED6
    initGPIO(E, 131, 0, GPIO_OUTPUT, GPIO_PUSHPULL, 0); // LED7
    initGPIO(A, 25, 0, GPIO_OUTPUT, GPIO_PUSHPULL, 0);  // LED8
    initGPIO(A, 26, 0, GPIO_OUTPUT, GPIO_PUSHPULL, 0);  // LED9
    initGPIO(A, 27, 0, GPIO_OUTPUT, GPIO_PUSHPULL, 0);  // LED10
    initGPIO(B, 60, 0, GPIO_OUTPUT, GPIO_PUSHPULL, 0);  // LED11
    initGPIO(B, 61, 0, GPIO_OUTPUT, GPIO_PUSHPULL, 0);  // LED12
    initGPIO(E, 157, 0, GPIO_OUTPUT, GPIO_PUSHPULL, 0); // LED13
    initGPIO(E, 158, 0, GPIO_OUTPUT, GPIO_PUSHPULL, 0); // LED14
    initGPIO(E, 159, 0, GPIO_OUTPUT, GPIO_PUSHPULL, 0); // LED15
    initGPIO(F, 160, 0, GPIO_OUTPUT, GPIO_PUSHPULL, 0); // LED16
    initGPIO(A, 0, 0, GPIO_OUTPUT, GPIO_PUSHPULL, 1);   // WIZNET Reset
    initGPIO(A, 1, 0, GPIO_OUTPUT, GPIO_PUSHPULL, 1);   // ESP8266 Reset
    initGPIO(A, 19, 0, GPIO_OUTPUT, GPIO_PUSHPULL, 1);  // SPIRAM Reset
    initGPIO(A, 29, 0, GPIO_OUTPUT, GPIO_PUSHPULL, 1);  // DRV8874 #1 DIR  Direction
    initGPIO(B, 32, 0, GPIO_OUTPUT, GPIO_PUSHPULL, 1);  // DRV8874 #2 DIR  Direction
    initGPIO(A, 9, 0, GPIO_OUTPUT, GPIO_PUSHPULL, 1);   // DAN28027  CS  Chip Select
    initGPIO(C, 66, 0, GPIO_OUTPUT, GPIO_PUSHPULL, 1);  // MPU9250  CS  Chip Select
    initGPIO(D, 125, 0, GPIO_OUTPUT, GPIO_PUSHPULL, 1); // WIZNET  CS  Chip Select
    setupGPIO(A, 4, 0, GPIO_INPUT, GPIO_PULLUP);        // PushButton 1
    setupGPIO(A, 5, 0, GPIO_INPUT, GPIO_PULLUP);        // PushButton 2
    setupGPIO(A, 6, 0, GPIO_INPUT, GPIO_PULLUP);        // PushButton 3
    setupGPIO(A, 7, 0, GPIO_INPUT, GPIO_PULLUP);        // PushButton 4
    setupGPIO(A, 8, 0, GPIO_INPUT, GPIO_PULLUP);        // Joy Stick PushButton
	
    // Clear all interrupts and initialize PIE vector table:
    // Disable CPU interrupts
    DINT;

    // Initialize the PIE control registers to their default state.
    // The default state is all PIE interrupts disabled and flags
    // are cleared.
    // This function is found in the F2837xD_PieCtrl.c file.
    InitPieCtrl();

    // Disable CPU interrupts and clear all CPU interrupt flags:
    IER = 0x0000;
    IFR = 0x0000;

    // Initialize the PIE vector table with pointers to the shell Interrupt
    // Service Routines (ISR).
    // This will populate the entire table, even if the interrupt
    // is not used in this example.  This is useful for debug purposes.
    // The shell ISR routines are found in F2837xD_DefaultIsr.c.
    // This function is found in F2837xD_PieVect.c.
    InitPieVectTable();

    // Interrupts that are used in this example are re-mapped to
    // ISR functions found within this project
    EALLOW;  // This is needed to write to EALLOW protected registers
    PieVectTable.TIMER0_INT = &cpu_timer0_isr;
    PieVectTable.TIMER1_INT = &cpu_timer1_isr;
    PieVectTable.TIMER2_INT = &cpu_timer2_isr;
    PieVectTable.SCIA_RX_INT = &RXAINT_recv_ready;
    PieVectTable.SCIC_RX_INT = &RXCINT_recv_ready;
    PieVectTable.SCID_RX_INT = &RXDINT_recv_ready;
    PieVectTable.SCIA_TX_INT = &TXAINT_data_sent;
    PieVectTable.SCIC_TX_INT = &TXCINT_data_sent;
    PieVectTable.SCID_TX_INT = &TXDINT_data_sent;
    PieVectTable.ADCD1_INT = &ADCD_ISR;
    PieVectTable.ADCA1_INT = &ADCA_ISR;
    PieVectTable.ADCB1_INT = &ADCB_ISR;

    PieVectTable.EMIF_ERROR_INT = &SWI_isr;
    EDIS;    // This is needed to disable write to EALLOW protected registers


    // Initialize the CpuTimers Device Peripheral. This function can be
    // found in F2837xD_CpuTimers.c
    InitCpuTimers();

    // Configure CPU-Timer 0, 1, and 2 to interrupt every second:
    // 200MHz CPU Freq, 1 second Period (in uSeconds)
//    ConfigCpuTimer(&CpuTimer0, 200, 1000000);
    ConfigCpuTimer(&CpuTimer1, 200, 31250);
    ConfigCpuTimer(&CpuTimer2, 200, 100000);  // 100ms clock

    // Enable CpuTimer Interrupt bit TIE
    CpuTimer0Regs.TCR.all = 0x4000;
    CpuTimer1Regs.TCR.all = 0x4000;
    CpuTimer2Regs.TCR.all = 0x4000;

    init_serial(&SerialA,115200,serialRXA);
//    init_serial(&SerialC,115200,serialRXC);
//    init_serial(&SerialD,115200,serialRXD);

    // Enable CPU int1 which is connected to CPU-Timer 0, CPU int13
    // which is connected to CPU-Timer 1, and CPU int 14, which is connected
    // to CPU-Timer 2:  int 12 is for the SWI.  
    IER |= M_INT1;  // ADCA1 ADCB1 ADCD1 TINT0
    IER |= M_INT8;  // SCIC SCID
    IER |= M_INT9;  // SCIA
    IER |= M_INT12;
    IER |= M_INT13;
    IER |= M_INT14;

    // Enable TINT0 in the PIE: Group 1 interrupt 7
    PieCtrlRegs.PIEIER1.bit.INTx7 = 1;
    // Enable ADCD1 in the PIE: Group 1 interrupt 6
    //PieCtrlRegs.PIEIER1.bit.INTx6 = 1;
    // Enable ADCA1 in the PIE: Group 1 interrupt 1
    PieCtrlRegs.PIEIER1.bit.INTx1 = 1;
    // Enable ADCB1 in the PIE: Group 1 interrupt 2
    PieCtrlRegs.PIEIER1.bit.INTx2 = 1;
	// Enable SWI in the PIE: Group 12 interrupt 9
    PieCtrlRegs.PIEIER12.bit.INTx9 = 1;
	
    // Enable global Interrupts and higher priority real-time debug events
    EINT;  // Enable Global interrupt INTM
    ERTM;  // Enable Global realtime interrupt DBGM

    active_filter = average_filter;
    filter_size = 5;

    // IDLE loop. Just sit and loop forever (optional):
    while(1)
    {
        //switchStates = ReadSwitches();
        if (UARTPrint == 1 ) {
            serial_printf(&SerialA,"Num adc:%ld Voltage: %.2f M %.2f\r\n", adcA_count, adcA0F, adcA1F);
            //serial_printf(&SerialA,"Switch states: %d\r\n",switchStates);
            //sliding_window_print(&SerialA, adcD_buffer);
            UARTPrint = 0;
        }
        //SetLEDRowsOnOff(switchStates);
    }
}
