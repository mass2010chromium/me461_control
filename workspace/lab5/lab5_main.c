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

void main() {
    // PLL, WatchDog, enable Peripheral Clocks
    // This example function is found in the F2837xD_SysCtrl.c file.
    InitSysCtrl();
    InitGpio();
    powerup_ADCs();
	
    initGPIO(31, 0, GPIO_OUTPUT, GPIO_PUSHPULL, 0); // Blue LED on LaunchPad
    initGPIO(34, 0, GPIO_OUTPUT, GPIO_PUSHPULL, 0); // Red LED on LaunchPad

    int i = 0;
    for (; i < 16; ++i) {
        // const int DISPLAY_LEDS[16] = {22, 94, 95, 97, 111, 130, 131, 25, 26, 27,
        //                                  60, 61, 157, 158, 159, 160};
        initGPIO(DISPLAY_LEDS[i], 0, GPIO_OUTPUT, GPIO_PUSHPULL, 0);
    }

    initGPIO(16, 5, GPIO_OUTPUT, GPIO_PUSHPULL, 0);  // Buzzer

    setupDAC(a, &dacA);
    setupDAC(b, &dacB);

    initGPIO(52, 0, GPIO_OUTPUT, GPIO_PUSHPULL, 0);  // Scope output

    initGPIO(0, 0, GPIO_OUTPUT, GPIO_PUSHPULL, 1);   // WIZNET Reset
    initGPIO(1, 0, GPIO_OUTPUT, GPIO_PUSHPULL, 1);   // ESP8266 Reset
    initGPIO(19, 0, GPIO_OUTPUT, GPIO_PUSHPULL, 1);  // SPIRAM Reset
    initGPIO(29, 0, GPIO_OUTPUT, GPIO_PUSHPULL, 1);  // DRV8874 #1 DIR  Direction
    initGPIO(32, 0, GPIO_OUTPUT, GPIO_PUSHPULL, 1);  // DRV8874 #2 DIR  Direction
    initGPIO(9, 0, GPIO_OUTPUT, GPIO_PUSHPULL, 1);   // DAN28027  CS  Chip Select
    initGPIO(66, 0, GPIO_OUTPUT, GPIO_PUSHPULL, 1);  // MPU9250  CS  Chip Select
    initGPIO(125, 0, GPIO_OUTPUT, GPIO_PUSHPULL, 1); // WIZNET  CS  Chip Select
    setupGPIO(4, 0, GPIO_INPUT, GPIO_PULLUP);        // PushButton 1
    setupGPIO(5, 0, GPIO_INPUT, GPIO_PULLUP);        // PushButton 2
    setupGPIO(6, 0, GPIO_INPUT, GPIO_PULLUP);        // PushButton 3
    setupGPIO(7, 0, GPIO_INPUT, GPIO_PULLUP);        // PushButton 4
    setupGPIO(8, 0, GPIO_INPUT, GPIO_PULLUP);        // Joy Stick PushButton
	
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
    ConfigCpuTimer(&CpuTimer0, 200, 20000);     // 20ms clock
    ConfigCpuTimer(&CpuTimer1, 200, 1000);      // 1ms clock
    ConfigCpuTimer(&CpuTimer2, 200, 10000);    // 10ms clock

    // Enable CpuTimer Interrupt bit TIE
    CpuTimer0Regs.TCR.all = 0x4000;
    CpuTimer1Regs.TCR.all = 0x4000;
    CpuTimer2Regs.TCR.all = 0x4000;

    init_serial(&SerialA,115200,serialRXA);
//    init_serial(&SerialC,115200,serialRXC);
//    init_serial(&SerialD,115200,serialRXD);

    GPIO_SetupPinMux(63, GPIO_MUX_CPU1, 15);
    GPIO_SetupPinMux(64, GPIO_MUX_CPU1, 15);
    GPIO_SetupPinMux(65, GPIO_MUX_CPU1, 15);
    SPI_setup(&spi_b, &SpibRegs, 65, 63, 64, 1000000, &PieVectTable.SPIB_RX_INT);
    SPI_start(&spi_b, 16, 0, SPI_no_callback);    // Setting for MPU9250, blocking mode

    imu.IO_mode = MPU9250_BLOCKING;
    imu.channel = &spi_b;
    imu.chipselect = 66;
    MPU9250_set_scaling(&imu, (250 * PI / 180.0), 4);

    MPU9250_set_write(&imu, 0x13, 0x00); // Writing offsets (Zero)
    MPU9250_set_write(&imu, 0x14, 0x00);
    MPU9250_set_write(&imu, 0x15, 0x00);
    MPU9250_set_write(&imu, 0x16, 0x00);
    MPU9250_set_write(&imu, 0x17, 0x00);
    MPU9250_set_write(&imu, 0x18, 0x00); // End offsets
    MPU9250_set_write(&imu, 0x19, 0x13); // sample rate divide
    MPU9250_set_write(&imu, 0x1A, 0x02); // More config (DLPF_CFG = 0b010: 92Hz bandwidth, 3.9ms delay)
    MPU9250_set_write(&imu, 0x1B, 0x00); // gyro config (250 dps, Fchoice = ~0b00 = 0b11)
    MPU9250_set_write(&imu, 0x1C, 0x08); // accel config (scale: 4g)
    MPU9250_set_write(&imu, 0x1D, 0x06); // accel config 2 (Low pass filter: 5Hz bandwidth, 66.96ms delay)
    MPU9250_set_write(&imu, 0x1E, 0x00); // Low Power Accel Output Data Rate
    MPU9250_set_write(&imu, 0x1F, 0x00); // Wake on Motion threshold

    MPU9250_set_write(&imu, 0x23, 0x00); // Disable fifos?
    MPU9250_set_write(&imu, 0x24, 0x40); // I2C stuff
    MPU9250_set_write(&imu, 0x25, 0x8C);
    MPU9250_set_write(&imu, 0x26, 0x02);
    MPU9250_set_write(&imu, 0x27, 0x88);
    MPU9250_set_write(&imu, 0x28, 0x0C);
    MPU9250_set_write(&imu, 0x29, 0x0A); // End I2C stuff

    MPU9250_set_write(&imu, 0x2A, 0x81); // One more I2C stuff

    MPU9250_set_write(&imu, 0x38, 0x01);
    MPU9250_set_write(&imu, 0x3A, 0x01);
    MPU9250_set_write(&imu, 0x64, 0x01);
    MPU9250_set_write(&imu, 0x67, 0x03);
    MPU9250_set_write(&imu, 0x6A, 0x20);
    MPU9250_set_write(&imu, 0x6B, 0x01);

    MPU9250_set_write(&imu, 0x75, 0x71);
    MPU9250_set_write(&imu, 0x76, 0x00); // WARN I added this lol
    MPU9250_set_write(&imu, 0x77, 0xEB);
    MPU9250_set_write(&imu, 0x78, 0x12);
    MPU9250_set_write(&imu, 0x79, 0x00); // WARN I added this lol
    MPU9250_set_write(&imu, 0x7A, 0x10);
    MPU9250_set_write(&imu, 0x7B, 0xFA);
    MPU9250_set_write(&imu, 0x7C, 0x00); // WARN I added this lol
    MPU9250_set_write(&imu, 0x7D, 0x21);
    MPU9250_set_write(&imu, 0x7E, 0x50);
    MPU9250_set_write(&imu, 0x7F, 0x00); // WARN I added this lol

    MPU9250_flush(&imu);

    MPU9250_calibrate(&imu, 1000);

    PieCtrlRegs.PIEIFR6.bit.INTx3 = 0;  // Clear interrupt manually? to avoid spurious interrupt
    SPI_start(&spi_b, 16, 0, MPU9250_SPIB_ISR);    // Setting for MPU9250, blocking mode
    imu.IO_mode = MPU9250_ASYNC;

    // Enable CPU int1 which is connected to CPU-Timer 0, CPU int13
    // which is connected to CPU-Timer 1, and CPU int 14, which is connected
    // to CPU-Timer 2:  int 12 is for the SWI.  
    IER |= M_INT1;  // ADCA1 ADCB1 ADCD1 TINT0
    IER |= M_INT6;  // SPIA, SPIB, SPIC
    IER |= M_INT8;  // SCIC SCID
    IER |= M_INT9;  // SCIA
    IER |= M_INT12;
    IER |= M_INT13;
    IER |= M_INT14;

    // Enable TINT0 in the PIE: Group 1 interrupt 7
    PieCtrlRegs.PIEIER1.bit.INTx7 = 1;
    // Enable ADCD1 in the PIE: Group 1 interrupt 6
    // PieCtrlRegs.PIEIER1.bit.INTx6 = 1;
    // Enable ADCA1 in the PIE: Group 1 interrupt 1
    // PieCtrlRegs.PIEIER1.bit.INTx1 = 1;
    // Enable ADCB1 in the PIE: Group 1 interrupt 2
    // PieCtrlRegs.PIEIER1.bit.INTx2 = 1;
    // Enable SPIB RX interrupt
    PieCtrlRegs.PIEIER6.bit.INTx3 = 1;
    // Enable SWI in the PIE: Group 12 interrupt 9
    PieCtrlRegs.PIEIER12.bit.INTx9 = 1;
	
    // Enable global Interrupts and higher priority real-time debug events
    EINT;  // Enable Global interrupt INTM
    ERTM;  // Enable Global realtime interrupt DBGM

    // IDLE loop. Just sit and loop forever (optional):
    i = 0;
    while(1)
    {
        //switchStates = ReadSwitches();
        if (UARTPrint == 1 ) {
            //serial_printf(&SerialA,"Num adc:%ld Voltage: %.2f M %.2f\r\n", adcA_count, adcA0F, adcA1F);
            //serial_printf(&SerialA,"Switch states: %d\r\n",switchStates);
            //serial_printf(&SerialA,"ADC: %.4f %.4f\r\n", (recv[1] & 0x0fff) * (3.3 / 4095),
            //                                              (recv[2] & 0x0fff) * (3.3 / 4095));
            serial_printf(&SerialA,"%f %f\r\n", angle_filtered, angle_raw);
            //serial_printf(&SerialA,"Reads: %ld, Angle: %.4f\r\n", CpuTimer1.InterruptCount, angle_filtered);
            //serial_printf(&SerialA,"Accel: %.4f %.4f %.4f\r\n", imu.accel_x, imu.accel_y, imu.accel_z);
            //serial_printf(&SerialA,"Gyro: %.4f %.4f %.4f\r\n", imu.gyro_x, imu.gyro_y, imu.gyro_z);
            //sliding_window_print(&SerialA, adcD_buffer);
            SetLEDRowsOnOff(i);
            ++i;
            UARTPrint = 0;
        }
        //SetLEDRowsOnOff(switchStates);
    }
}
