//#############################################################################
// FILE:   CPU1_main.c
//
// TITLE:  
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
#include "f28379dSerial.h"
#include "LEDPatterns.h"
#include "song.h"
#include "dsp.h"
#include "fpu32/fpu_rfft.h"
// For IPC
#include "F2837xD_Ipc_drivers.h"

#define PI          3.1415926535897932384626433832795
#define TWOPI       6.283185307179586476925286766559
#define HALFPI      1.5707963267948966192313216916398
#define FEETINONEMETER 3.28083989501312
#define MIN(A,B)        (((A) < (B)) ? (A) : (B));


// Interrupt Service Routines predefinition
__interrupt void cpu_timer0_isr(void);
__interrupt void cpu_timer1_isr(void);
__interrupt void cpu_timer2_isr(void);
__interrupt void SWI_isr(void);

// Count variables
uint32_t numTimer0calls = 0;
uint32_t numSWIcalls = 0;
uint32_t numRXA = 0;
uint16_t UARTPrint = 0;
uint32_t numCPU2COMs = 0;
float receivefloats[4] = {0,0,0,0};
float sendfloats[4] = {0,0,0,0};
uint32_t timecounter = 0;

void serialRXA(serial_t *s, char data);

//void serialRXD(serial_t *s, char data);//for lidar data reading

//=======================================================CODE FOR INITIALIZATION AND FUNCTION PREDEFINITIONS ==================================================================

__interrupt void CPU2toCPU1IPC0(void);


float *cpu2tocpu1; //float pointer for the location CPU2 will give data to CPU1
float *cpu1tocpu2; //float pointer for the location CPU1 will give data to CPU2


void main(void)
{


    // PLL, WatchDog, enable Peripheral Clocks
    // This example function is found in the F2837xD_SysCtrl.c file.
    InitSysCtrl();

    //     Comment this when use CCS for debugging
//            #ifdef _FLASH
//                // Send boot command to allow the CPU2 application to begin execution
                IPCBootCPU2(C1C2_BROM_BOOTMODE_BOOT_FROM_FLASH);
//            #else
//                // Send boot command to allow the CPU2 application to begin execution
//                IPCBootCPU2(C1C2_BROM_BOOTMODE_BOOT_FROM_RAM);
//            #endif
//  Or when you want to run CPU2 from its flash you free run CPU2 and just run IPCBootCPU2 from Flash command.  Actually I do not know when you need boot from RAM ???


    InitGpio();

    // Blue LED on LuanchPad
    GPIO_SetupPinMux(31, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(31, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPASET.bit.GPIO31 = 1;

    // LED1 and PWM Pin
    GPIO_SetupPinMux(22, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(22, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPACLEAR.bit.GPIO22 = 1;

    // LED2
    GPIO_SetupPinMux(94, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(94, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPCCLEAR.bit.GPIO94 = 1;

    // LED3
    GPIO_SetupPinMux(95, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(95, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPCCLEAR.bit.GPIO95 = 1;

    // LED4
    GPIO_SetupPinMux(97, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(97, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPDCLEAR.bit.GPIO97 = 1;

    // LED5
    GPIO_SetupPinMux(111, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(111, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPDCLEAR.bit.GPIO111 = 1;

    // LED6
    GPIO_SetupPinMux(130, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(130, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPECLEAR.bit.GPIO130 = 1;

    // LED7
    GPIO_SetupPinMux(131, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(131, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPECLEAR.bit.GPIO131 = 1;

    // LED8
    GPIO_SetupPinMux(25, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(25, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPACLEAR.bit.GPIO25 = 1;

    // LED9
    GPIO_SetupPinMux(26, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(26, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPACLEAR.bit.GPIO26 = 1;

    // LED10
    GPIO_SetupPinMux(27, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(27, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPACLEAR.bit.GPIO27 = 1;

    //ESP8266 Reset
    GPIO_SetupPinMux(1, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(1, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPASET.bit.GPIO1 = 1;

    //SPIRAM  CS  Chip Select
    GPIO_SetupPinMux(19, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(19, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPASET.bit.GPIO19 = 1;

    //DRV8874 #1 DIR  Direction
    GPIO_SetupPinMux(29, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(29, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPASET.bit.GPIO29 = 1;

    //DRV8874 #2 DIR  Direction
    GPIO_SetupPinMux(32, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(32, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPBSET.bit.GPIO32 = 1;

    //DAN28027  CS  Chip Select
    GPIO_SetupPinMux(9, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(9, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPASET.bit.GPIO9 = 1;

    //MPU9250  CS  Chip Select
    GPIO_SetupPinMux(66, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(66, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPCSET.bit.GPIO66 = 1;

    //WIZNET  CS  Chip Select
    GPIO_SetupPinMux(125, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(125, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPDSET.bit.GPIO125 = 1;

    //PushButton 1
    GPIO_SetupPinMux(4, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(4, GPIO_INPUT, GPIO_PULLUP);

    //PushButton 2
    GPIO_SetupPinMux(5, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(5, GPIO_INPUT, GPIO_PULLUP);

    //PushButton 3
    GPIO_SetupPinMux(6, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(6, GPIO_INPUT, GPIO_PULLUP);

    //PushButton 4
    GPIO_SetupPinMux(7, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(7, GPIO_INPUT, GPIO_PULLUP);

    //Joy Stick Pushbutton
    GPIO_SetupPinMux(8, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(8, GPIO_INPUT, GPIO_PULLUP);

    //gpio52
    GPIO_SetupPinMux(52, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(52, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPBSET.bit.GPIO52 = 1;


    // Red LED on LaunchPad, change ownership to CPU2
    GPIO_SetupPinMux(34, GPIO_MUX_CPU2, 0);
    GPIO_SetupPinOptions(34, GPIO_OUTPUT, GPIO_PUSHPULL);

    // LED11 to LED 16 is the last column on the board, change ownership to CPU2-----------------
    // LED11
    GPIO_SetupPinMux(60, GPIO_MUX_CPU2, 0);
    GPIO_SetupPinOptions(60, GPIO_OUTPUT, GPIO_PUSHPULL);

    // LED12
    GPIO_SetupPinMux(61, GPIO_MUX_CPU2, 0);
    GPIO_SetupPinOptions(61, GPIO_OUTPUT, GPIO_PUSHPULL);

    // LED13
    GPIO_SetupPinMux(157, GPIO_MUX_CPU2, 0);
    GPIO_SetupPinOptions(157, GPIO_OUTPUT, GPIO_PUSHPULL);

    // LED14
    GPIO_SetupPinMux(158, GPIO_MUX_CPU2, 0);
    GPIO_SetupPinOptions(158, GPIO_OUTPUT, GPIO_PUSHPULL);

    // LED15
    GPIO_SetupPinMux(159, GPIO_MUX_CPU2, 0);
    GPIO_SetupPinOptions(159, GPIO_OUTPUT, GPIO_PUSHPULL);

    // LED16
    GPIO_SetupPinMux(160, GPIO_MUX_CPU2, 0);
    GPIO_SetupPinOptions(160, GPIO_OUTPUT, GPIO_PUSHPULL);
    //--------------------------------------------------------------------------------------------

    //wiznet related GPIO, change ownership to CPU2------------------------------------
    //wiznet reset
    GPIO_SetupPinMux(0, GPIO_MUX_CPU2, 0);
    GPIO_SetupPinOptions(0, GPIO_OUTPUT, GPIO_PUSHPULL);

    GPIO_SetupPinMux(67, GPIO_MUX_CPU2, 0);
    GPIO_SetupPinOptions(67, GPIO_OUTPUT, GPIO_PUSHPULL);

    EALLOW;
    GpioCtrlRegs.GPDPUD.bit.GPIO122 = 0; // Enable Pull-ups on SPI PINs Recommended by TI for SPI Pins
    GpioCtrlRegs.GPDPUD.bit.GPIO123 = 0;
    GpioCtrlRegs.GPDPUD.bit.GPIO124 = 0;
    GpioCtrlRegs.GPDQSEL2.bit.GPIO122 = 3; // Set prequalifier for SPI PINS
    GpioCtrlRegs.GPDQSEL2.bit.GPIO123 = 3; // The prequalifier eliminates short noise spikes
    GpioCtrlRegs.GPDQSEL2.bit.GPIO124 = 3; // by making sure the serial pin stays low for 3 clock periods.
    EDIS;

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

    //??????????????? should this be here or up above?????///
    // Comment this when use CCS for debugging
//        #ifdef _FLASH
//            // Send boot command to allow the CPU2 application to begin execution
//            IPCBootCPU2(C1C2_BROM_BOOTMODE_BOOT_FROM_FLASH);
//        #else
//            // Send boot command to allow the CPU2 application to begin execution
//            IPCBootCPU2(C1C2_BROM_BOOTMODE_BOOT_FROM_RAM);
//        #endif

    // Interrupts that are used in this example are re-mapped to
    // ISR functions found within this project
    EALLOW;  // This is needed to write to EALLOW protected registers
    PieVectTable.TIMER0_INT = &cpu_timer0_isr;
    PieVectTable.TIMER1_INT = &cpu_timer1_isr;
    PieVectTable.TIMER2_INT = &cpu_timer2_isr;
    PieVectTable.SCIA_RX_INT = &RXAINT_recv_ready;
//    PieVectTable.SCIC_RX_INT = &RXCINT_recv_ready;
    PieVectTable.SCID_RX_INT = &RXDINT_recv_ready;
    PieVectTable.SCIA_TX_INT = &TXAINT_data_sent;
//    PieVectTable.SCIC_TX_INT = &TXCINT_data_sent;
    PieVectTable.SCID_TX_INT = &TXDINT_data_sent;
    PieVectTable.IPC0_INT = &CPU2toCPU1IPC0;
    PieVectTable.EMIF_ERROR_INT = &SWI_isr;
    EDIS;    // This is needed to disable write to EALLOW protected registers


    // Initialize the CpuTimers Device Peripheral. This function can be
    // found in F2837xD_CpuTimers.c
    InitCpuTimers();

    // Configure CPU-Timer 0, 1, and 2 to interrupt every second:
    // 200MHz CPU Freq, 1 second Period (in uSeconds)
    ConfigCpuTimer(&CpuTimer0, 200, 500000); // 500ms
    ConfigCpuTimer(&CpuTimer1, 200, 4000);
    ConfigCpuTimer(&CpuTimer2, 200, 222000);

    // Enable CpuTimer Interrupt bit TIE
    CpuTimer0Regs.TCR.all = 0x4000;
    CpuTimer1Regs.TCR.all = 0x4000;
    CpuTimer2Regs.TCR.all = 0x4000;

    init_serial(&SerialA,115200,serialRXA);
    //init_serial(&SerialD,115200,serialRXD);

    // Enable CPU int1 which is connected to CPU-Timer 0, CPU int13
    // which is connected to CPU-Timer 1, and CPU int 14, which is connected
    // to CPU-Timer 2:  int 12 is for the SWI.  
    IER |= M_INT1;
    IER |= M_INT8;  // SCIC SCID
    IER |= M_INT9;  // SCIA
    IER |= M_INT12;
    IER |= M_INT13;
    IER |= M_INT14;


    //need to acknowledge IPC before use
    IpcRegs.IPCACK.bit.IPC0 = 1;
    PieCtrlRegs.PIEACK.all = PIEACK_GROUP1;

    // Enable TINT0 in the PIE: Group 1 interrupt 7
    PieCtrlRegs.PIEIER1.bit.INTx7 = 1;
    // Enable SWI in the PIE: Group 12 interrupt 9
    PieCtrlRegs.PIEIER12.bit.INTx9 = 1;
    //IPC
    PieCtrlRegs.PIEIER1.bit.INTx13 = 1;

    //location of cpu2tocpu1 ram
    cpu2tocpu1 = (float*) 0x3F800;
    cpu1tocpu2 = (float*) 0x3FC00;

    // SCIC setup for CPU2
    GPIO_SetupPinMux(139, GPIO_MUX_CPU1, 6);
    GPIO_SetupPinOptions(139, GPIO_INPUT, GPIO_PULLUP);
    GPIO_SetupPinMux(56, GPIO_MUX_CPU1, 6);
    GPIO_SetupPinOptions(56, GPIO_OUTPUT, GPIO_PUSHPULL);


    uint32_t clk;
    uint32_t baud = 115200;
    ScicRegs.SCICTL1.bit.SWRESET = 0;       // init SCI state machines and opt flags
    ScicRegs.SCICCR.all = 0x0;
    ScicRegs.SCICTL1.all = 0x0;
    ScicRegs.SCICTL2.all = 0x0;
    ScicRegs.SCIPRI.all = 0x0;
    clk = LSPCLK_HZ;                    // set baud rate
    clk /= baud*8;
    clk--;
    ScicRegs.SCILBAUD.all = clk & 0xFF;
    ScicRegs.SCIHBAUD.all = (clk >> 8) & 0xFF;

    ScicRegs.SCICCR.bit.SCICHAR = 0x7;      // (8) 8 bits per character
    ScicRegs.SCICCR.bit.PARITYENA = 0;      // (N) disable party calculation
    ScicRegs.SCICCR.bit.STOPBITS = 0;       // (1) transmit 1 stop bit
    ScicRegs.SCICCR.bit.LOOPBKENA = 0;      // disable loopback test
    ScicRegs.SCICCR.bit.ADDRIDLE_MODE = 0;  // idle-line mode (non-multiprocessor SCI comm)

    ScicRegs.SCIFFCT.bit.FFTXDLY = 0;       // TX: zero-delay

    ScicRegs.SCIFFTX.bit.SCIFFENA = 1;      // enable SCI fifo enhancements
    ScicRegs.SCIFFTX.bit.TXFIFORESET = 0;
    ScicRegs.SCIFFTX.bit.TXFFIL = 0x0;// TX: fifo interrupt at all levels   ???? is this correct
    ScicRegs.SCIFFTX.bit.TXFFINTCLR = 1;  // TX: clear interrupt flag
    ScicRegs.SCIFFTX.bit.TXFFIENA = 0;      // TX: disable fifo interrupt
    ScicRegs.SCIFFTX.bit.TXFIFORESET = 1;

    ScicRegs.SCIFFRX.bit.RXFIFORESET = 0;   // RX: fifo reset
    ScicRegs.SCIFFRX.bit.RXFFINTCLR = 1;    // RX: clear interrupt flag
    ScicRegs.SCIFFRX.bit.RXFFIENA = 1;      // RX: enable fifo interrupt
    ScicRegs.SCIFFRX.bit.RXFFIL = 0x1;      // RX: fifo interrupt
    ScicRegs.SCIFFRX.bit.RXFIFORESET = 1;   // RX: re-enable fifo

    ScicRegs.SCICTL2.bit.RXBKINTENA = 0;    // disable receiver/error interrupt
    ScicRegs.SCICTL2.bit.TXINTENA = 0;      // disable transmitter interrupt

    ScicRegs.SCICTL1.bit.TXWAKE = 0;
    ScicRegs.SCICTL1.bit.SLEEP = 0;         // disable sleep mode
    ScicRegs.SCICTL1.bit.RXENA = 1;         // enable SCI receiver
    ScicRegs.SCICTL1.bit.RXERRINTENA = 0;   // disable receive error interrupt
    ScicRegs.SCICTL1.bit.TXENA = 1;         // enable SCI transmitter
    ScicRegs.SCICTL1.bit.SWRESET = 1;       // re-enable SCI


    //for CPU2 to use SCIC and or SPIC
    EALLOW;
//            DevCfgRegs.CPUSEL6.bit.SPI_C = 1;  //SPI C connected to CPU2
            DevCfgRegs.CPUSEL5.bit.SCI_C = 1;  //SCI C connected to CPU2
    EDIS;

    // Enable global Interrupts and higher priority real-time debug events
    EINT;  // Enable Global interrupt INTM
    ERTM;  // Enable Global realtime interrupt DBGM



    // IDLE loop. Just sit and loop forever (optional):
    while(1)
    {
        if (UARTPrint == 1 ) {
            serial_printf(&SerialA,"RXA %ld numCPU2COMs %ld %.3f %.3f %.3f %.3f\r\n",numRXA,numCPU2COMs,receivefloats[0],receivefloats[1],receivefloats[2],receivefloats[3]);
            UARTPrint = 0;
        }


    }
}


//==================================================CODE FOR FUNCTIONS========================================================================
//IPC
__interrupt void CPU2toCPU1IPC0(void){
    GpioDataRegs.GPBTOGGLE.bit.GPIO52 = 1;
    GpioDataRegs.GPETOGGLE.bit.GPIO131 = 1;//led7
    //put data from cpu2 into cpu2tocpu1 array
    int i;
    for (i=0;i<4;i++) {
        receivefloats[i] = cpu2tocpu1[i];
    }
    numCPU2COMs++;
    UARTPrint = 1;
    IpcRegs.IPCACK.bit.IPC0 = 1;
    PieCtrlRegs.PIEACK.all = PIEACK_GROUP1;
}


// cpu_timer1_isr - CPU Timer1 ISR
__interrupt void cpu_timer1_isr(void)
{
    CpuTimer1.InterruptCount++;

}

//--------------------------------------------------end of command robot to move to the desired point set up---------------------------------------------------------

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

    // Blink LaunchPad Blue LED
    GpioDataRegs.GPATOGGLE.bit.GPIO31 = 1;

    // Acknowledge this interrupt to receive more interrupts from group 1
    PieCtrlRegs.PIEACK.all = PIEACK_GROUP1;
}

// cpu_timer2_isr CPU Timer2 ISR
__interrupt void cpu_timer2_isr(void)
{

    CpuTimer2.InterruptCount++;

    cpu1tocpu2[0] = 10*sin(2*PI*0.1*(timecounter*0.222));
    cpu1tocpu2[1] = 12*cos(2*PI*0.1*(timecounter*0.222));
    cpu1tocpu2[2] = 14*sin(2*PI*0.1*(timecounter*0.222));
    cpu1tocpu2[3] = 16*cos(2*PI*0.1*(timecounter*0.222));
    timecounter++;

    IpcRegs.IPCSET.bit.IPC0 = 1;
}

// This function is called each time a char is received over UARTA.
void serialRXA(serial_t *s, char data) {
    numRXA ++;
}
