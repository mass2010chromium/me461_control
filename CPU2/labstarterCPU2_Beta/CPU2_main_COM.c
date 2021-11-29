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
#include "f28379dSerialCPU2.h"

#include "dsp.h"
#include "fpu32/fpu_rfft.h"

#define PI          3.1415926535897932384626433832795
#define TWOPI       6.283185307179586476925286766559
#define HALFPI      1.5707963267948966192313216916398


// Interrupt Service Routines predefinition
__interrupt void cpu_timer0_isr(void);
__interrupt void cpu_timer1_isr(void);
__interrupt void cpu_timer2_isr(void);
__interrupt void SWI_isr(void);
__interrupt void CPU1toCPU2IPC0(void);


void serialRXC(serial_t *s, char data);

// Count variables
uint32_t numTimer0calls = 0;
uint32_t numSWIcalls = 0;
float *cpu2tocpu1;
float *cpu1tocpu2;
uint32_t numRXC = 0;
uint16_t UARTPrint = 0;
uint32_t numCPU1COMs = 0;
float CPU2receivefloats[4] = {0,0,0,0};
float CPU2sendfloats[4] = {0,0,0,0};

uint32_t timecounter = 0;

char sendstring[30] = "This is a string";





void main(void)
{
    // PLL, WatchDog, enable Peripheral Clocks
    // This example function is found in the F2837xD_SysCtrl.c file.
    InitSysCtrl();

    GpioDataRegs.GPCSET.bit.GPIO67 = 1;


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
    PieVectTable.SCIC_RX_INT = &RXCINT_recv_ready;
    PieVectTable.SCIC_TX_INT = &TXCINT_data_sent;
    PieVectTable.IPC0_INT = &CPU1toCPU2IPC0;
    PieVectTable.EMIF_ERROR_INT = &SWI_isr;
    EDIS;    // This is needed to disable write to EALLOW protected registers


    // Initialize the CpuTimers Device Peripheral. This function can be
    // found in F2837xD_CpuTimers.c
    InitCpuTimers();

    // Configure CPU-Timer 0, 1, and 2 to interrupt every second:
    // 200MHz CPU Freq, 1 second Period (in uSeconds)
    ConfigCpuTimer(&CpuTimer0, 200, 137000); // 137ms
    ConfigCpuTimer(&CpuTimer1, 200, 1000);
    ConfigCpuTimer(&CpuTimer2, 200, 40000);

    // Enable CpuTimer Interrupt bit TIE
    CpuTimer0Regs.TCR.all = 0x4000;
    CpuTimer1Regs.TCR.all = 0x4000;
    CpuTimer2Regs.TCR.all = 0x4000;

    init_serial(&SerialC,115200,serialRXC);
    // Enable CPU int1 which is connected to CPU-Timer 0, CPU int13
    // which is connected to CPU-Timer 1, and CPU int 14, which is connected
    // to CPU-Timer 2:  int 12 is for the SWI.  
    IER |= M_INT1;
    IER |= M_INT8;//scic
    IER |= M_INT12;
    IER |= M_INT13;
    IER |= M_INT14;

    // Enable TINT0 in the PIE: Group 1 interrupt 7
    PieCtrlRegs.PIEIER1.bit.INTx7 = 1;

    // Enable SWI in the PIE: Group 12 interrupt 9
    PieCtrlRegs.PIEIER12.bit.INTx9 = 1;
    //IPC0
    PieCtrlRegs.PIEIER1.bit.INTx13 = 1;

    cpu2tocpu1 = (float*) 0x3F800;
    cpu1tocpu2 = (float*) 0x3FC00;


    // Enable global Interrupts and higher priority real-time debug events
    EINT;  // Enable Global interrupt INTM
    ERTM;  // Enable Global realtime interrupt DBGM


    // IDLE loop. Just sit and loop forever (optional):
    while(1)
    {
        if (UARTPrint == 1 ) {
            //serial_send(&SerialC, sendstring, 18);
            serial_printf(&SerialC,"RXC %ld numCPU2COMs %ld %.3f %.3f %.3f %.3f\r\n",numRXC,numCPU1COMs,CPU2receivefloats[0],CPU2receivefloats[1],CPU2receivefloats[2],CPU2receivefloats[3]);
            UARTPrint = 0;
        }


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


//IPC
__interrupt void CPU1toCPU2IPC0(void){
    //put data from cpu2 into cpu2tocpu1 array
    int i;
    for (i=0;i<4;i++) {
        CPU2receivefloats[i] = cpu1tocpu2[i];
    }
    numCPU1COMs++;
    UARTPrint = 1;
    IpcRegs.IPCACK.bit.IPC0 = 1;
    PieCtrlRegs.PIEACK.all = PIEACK_GROUP1;
}

// cpu_timer0_isr - CPU Timer0 ISR
__interrupt void cpu_timer0_isr(void)
{
    GpioDataRegs.GPASET.bit.GPIO7 = 1;

    CpuTimer0.InterruptCount++;

    numTimer0calls++;

    //    if ((numTimer0calls%50) == 0) {
    //        PieCtrlRegs.PIEIFR12.bit.INTx9 = 1;  // Manually cause the interrupt for the SWI
    //    }



    cpu2tocpu1[0] = sin(2*PI*0.1*(timecounter*0.137));
    cpu2tocpu1[1] = 2*cos(2*PI*0.1*(timecounter*0.137));
    cpu2tocpu1[2] = 3*sin(2*PI*0.1*(timecounter*0.137));
    cpu2tocpu1[3] = 4*cos(2*PI*0.1*(timecounter*0.137));
    timecounter++;

    IpcRegs.IPCSET.bit.IPC0 = 1;

    GpioDataRegs.GPCCLEAR.bit.GPIO67 = 1;



    GpioDataRegs.GPBTOGGLE.bit.GPIO34 = 1;
    GpioDataRegs.GPBTOGGLE.bit.GPIO61 = 1;//led12
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
    CpuTimer2.InterruptCount++;
}

// This function is called each time a char is received over UARTC.
void serialRXC(serial_t *s, char data) {
    numRXC ++;
}
