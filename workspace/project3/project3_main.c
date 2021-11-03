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

    uint32_t i = 0;
    for (; i < 16; ++i) {
        initGPIO(DISPLAY_LEDS[i], 0, GPIO_OUTPUT, GPIO_PUSHPULL, 0);
    }

    initGPIO(16, 0, GPIO_OUTPUT, GPIO_PUSHPULL, 0);  // Buzzer

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

    I2C_init(&I2cbRegs);

    // Initialize the CpuTimers Device Peripheral. This function can be
    // found in F2837xD_CpuTimers.c
    InitCpuTimers();

    // Configure CPU-Timer 0, 1, and 2 to interrupt every second:
    // 200MHz CPU Freq, 1 second Period (in uSeconds)
    ConfigCpuTimer(&CpuTimer0, 200, 10000);     // 10ms clock
    ConfigCpuTimer(&CpuTimer1, 200, 10000);     // 1ms clock (unused)
    ConfigCpuTimer(&CpuTimer2, 200, 250000);    // 250ms clock

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

    PieCtrlRegs.PIEIFR6.bit.INTx3 = 0;  // Clear interrupt manually? to avoid spurious interrupt

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
    uint16_t readwrite_buf[8]; // Big enough for everything

    const uint16_t DAN_ADDRESS = 0x7D;
    const uint16_t BQ32000_ADDRESS = 0b1101000; // 0x68
    const char* days[8] = {
        "null",
        "Sunday",
        "Monday",
        "Tuesday",
        "Wednesday",
        "Thursday",
        "Friday",
        "Saturday"
    };
    const char* day_short = "UMTWFRS";    // single character day of the week repr.
    // Communication results (return values for debugging? purposes)
    uint16_t res;
    uint16_t res2;
    uint16_t res3;
    // Servo command to send.
    uint16_t servo_cmd;
    // ADC readings from the DAN
    float adc1_reading;
    float adc2_reading;
    // BQ readings
    uint16_t seconds;
    uint16_t minutes;
    uint16_t hours;
    uint16_t day;
    uint16_t date;
    uint16_t month;
    uint16_t year;
    uint16_t j = 0;
    while(1)
    {
        //switchStates = ReadSwitches();
        if (UARTPrint) {
            serial_printf(&SerialA, "%s", bs);
            uint16_t nprint1 = serial_printf(&SerialA, "Today is %s %02u/%02u/%02u %02u:%02u:%02u\r\n",
                    days[day], month, date, year, hours, minutes, seconds);
            uint16_t nprint2 = serial_printf(&SerialA, "pwm %4u adc (%.3f, %.3f) status %u%u%u\r\n",
                    servo_cmd, adc1_reading, adc2_reading, res, res2, res3);
            serial_printf(&SerialA, "%s", buf);
            UARTPrint = 0;
            ++j;
            SetLEDRowsOnOff(j);
        }
        if (I2C_command) {
            I2C_command = 0;

            // Use the counter to create a triangle wave.
            // `servo_cmd` ranges from 0 to 2999, and gets reflected when it hits 1500.
            servo_cmd = (i<<4) % 3000;
            if (servo_cmd > 1500) servo_cmd = 3000 - servo_cmd;

            lsbmsb_convert(readwrite_buf+0, servo_cmd + 1500);  // writes into the first two words (lsb, msb).
            lsbmsb_convert(readwrite_buf+2, servo_cmd + 1500);  // writes into the next two words (lsb, msb).
            res = write_to_I2C(&I2cbRegs, DAN_ADDRESS, 4, 4, readwrite_buf);    // Write the four words out to the DAN, starting at register 4.
            res2 = read_from_I2C(&I2cbRegs, DAN_ADDRESS, 0, 4, readwrite_buf);  // Read four words frorm the DAN, starting at register 0.
            adc1_reading = DAN_ADC_TO_VOLTS * lsbmsb_parse(readwrite_buf + 0);  // Parse the first two words (lsb, msb) to make one adc reading.
            adc2_reading = DAN_ADC_TO_VOLTS * lsbmsb_parse(readwrite_buf + 2);  // Parse the next two words (lsb, msb) to make one adc reading.
            res3 = read_from_I2C(&I2cbRegs, BQ32000_ADDRESS, 0, 7, readwrite_buf);  // Read 7 words from the BQ, stating at register 0.
            seconds = from_bcd(readwrite_buf[0] & 0x7f);    // bottom 7 bits (0-49)
            minutes = from_bcd(readwrite_buf[1] & 0x7f);    // bottom 7 bits (0-59)
            hours = from_bcd(readwrite_buf[2] & 0x3f);      // bottom 6 bits (0-23)
            day = readwrite_buf[3] & 0x07;                  // bottom 3 bits (1-7). No conversion needed
            date = from_bcd(readwrite_buf[4] & 0x3f);       // bottom 6 bits (1-31)
            month = from_bcd(readwrite_buf[5] & 0x1f);      // bottom 5 bits (1-12)
            year = from_bcd(readwrite_buf[6] & 0xff);       // all bits (0-99)
            ++i;
        }
        else if (serial_change) {
            // Refresh the line entry at the bottom.
            serial_printf(&SerialA, "%s", bs);
            serial_change = 0;
            serial_printf(&SerialA, "%s", buf);

            // Parse a date from user input if required. Quite inflexible, too lazy.
            if (date_parse) {
                readwrite_buf[0] = parse_hex(parse+17);   // sec
                readwrite_buf[1] = parse_hex(parse+14);   // min
                readwrite_buf[2] = parse_hex(parse+11);   // hour
                readwrite_buf[3] = (strchr(day_short, parse[0]) - day_short) + 1; // day (UMTWRFS)
                readwrite_buf[4] = parse_hex(parse+5);    // date
                readwrite_buf[5] = parse_hex(parse+2);    // month
                readwrite_buf[6] = parse_hex(parse+8);    // year
                uint16_t res = write_to_I2C(&I2cbRegs, BQ32000_ADDRESS, 0, 7, readwrite_buf);   // Write 7 bytes back out to the BQ chip
                serial_printf(&SerialA, "\r\nSet time: status %d\r\n", res);
                head = HEAD_ZERO;
                bs[head] = 0;
                buf[head] = 0;
                date_parse = 0;
            }
        }
    }
}
