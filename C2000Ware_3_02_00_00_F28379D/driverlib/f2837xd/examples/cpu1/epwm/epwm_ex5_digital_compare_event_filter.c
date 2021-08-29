//#############################################################################
//
// FILE:   epwm_ex5_digital_compare_event_filter.c
//
// TITLE:  ePWM Using Digital Compare Submodule with
//         Event Filter (Blanking Window).
//
//! \addtogroup driver_example_list
//! <h1>ePWM Digital Compare Event Filter Blanking Window</h1>
//!
//! This example configures ePWM1 as follows
//!  - ePWM1 with DCAEVT1 forcing the ePWM output LOW
//!  - GPIO25 is used as the input to the INPUT XBAR INPUT1
//!  - INPUT1 (from INPUT XBAR) is used as the source for DCAEVT1
//!  - GPIO25's PULL-UP resistor is enabled, in order to test the trip, PULL
//!    this pin to GND
//!
//!  - ePWM1 with DCBEVT1 forcing the ePWM output LOW
//!  - GPIO25 is used as the input to the INPUT XBAR INPUT1
//!  - INPUT1 (from INPUT XBAR) is used as the source for DCAEVT1
//!  - GPIO25's PULL-UP resistor is enabled, in order to test the trip, PULL
//!    this pin to GND
//!  - DCBEVT1 uses the filtered version of DCBEVT1
//!  - The DCFILT signal uses the blanking window to ignore the DCBEVT1
//!    for the duration of DC Blanking window
//!
//! \b External \b Connections \n
//! - GPIO0 EPWM1A
//! - GPIO1 EPWM1B
//! - GPIO25 TRIPIN1, pull this pin low to trip the ePWM
//!
//! \b Watch \b Variables \n
//! - None.
//
//
//#############################################################################
// $TI Release: F2837xD Support Library v3.10.00.00 $
// $Release Date: Tue May 26 17:13:46 IST 2020 $
// $Copyright:
// Copyright (C) 2013-2020 Texas Instruments Incorporated - http://www.ti.com/
//
// Redistribution and use in source and binary forms, with or without 
// modification, are permitted provided that the following conditions 
// are met:
// 
//   Redistributions of source code must retain the above copyright 
//   notice, this list of conditions and the following disclaimer.
// 
//   Redistributions in binary form must reproduce the above copyright
//   notice, this list of conditions and the following disclaimer in the 
//   documentation and/or other materials provided with the   
//   distribution.
// 
//   Neither the name of Texas Instruments Incorporated nor the names of
//   its contributors may be used to endorse or promote products derived
//   from this software without specific prior written permission.
// 
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS 
// "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT 
// LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
// A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT 
// OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, 
// SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT 
// LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
// DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
// THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT 
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE 
// OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
// $
//#############################################################################

//
// Included Files
//
#include "driverlib.h"
#include "device.h"

//
// Globals
//
uint32_t  epwm1TZIntCount;

//
// Function Prototypes
//
void initEPWM1(void);
void initTZGPIO(void);
void initEPWMGPIO(void);
__interrupt void epwm1TZISR(void);

void main(void)
{
    //
    // Initialize global variables
    //
    epwm1TZIntCount = 0U;

    //
    // Initialize device clock and peripherals
    //
    Device_init();

    //
    // Disable pin locks and enable internal pull-ups.
    //
    Device_initGPIO();

    //
    // Initialize PIE and clear PIE registers. Disables CPU interrupts.
    //
    Interrupt_initModule();

    //
    // Initialize the PIE vector table with pointers to the shell Interrupt
    // Service Routines (ISR).
    //
    Interrupt_initVectorTable();

    //
    // Interrupts that are used in this example are re-mapped to ISR functions
    // found within this file.
    //
    Interrupt_register(INT_EPWM1_TZ, &epwm1TZISR);

    //
    // Configure ePWM1, and TZ GPIOs
    //
    initEPWMGPIO();
    initTZGPIO();

    //
    // Disable sync(Freeze clock to PWM as well). GTBCLKSYNC is applicable
    // only for multiple core devices. Uncomment the below statement if
    // applicable.
    //
    // SysCtl_disablePeripheral(SYSCTL_PERIPH_CLK_GTBCLKSYNC);
    SysCtl_disablePeripheral(SYSCTL_PERIPH_CLK_TBCLKSYNC);

    //
    // Initialize ePWM1 with Digital Compare and Blanking window
    //
    initEPWM1();

    //
    // Enable sync and clock to PWM
    //
    SysCtl_enablePeripheral(SYSCTL_PERIPH_CLK_TBCLKSYNC);

    //
    // Enable interrupts required for this example
    //
    Interrupt_enable(INT_EPWM1_TZ);

    //
    // Enable Global Interrupt (INTM) and real time interrupt (DBGM)
    //
    EINT;
    ERTM;

    //
    // IDLE loop. Just sit and loop forever (optional):
    //
    for(;;)
    {
        NOP;
    }
}

//
// epwm1TZISR - ePWM1 TZ ISR
//
__interrupt void epwm1TZISR(void)
{
    epwm1TZIntCount++;

    //
    // To re-enable the Interrupts
    //
    EPWM_clearTripZoneFlag(EPWM1_BASE,
                           EPWM_TZ_FLAG_DCAEVT1 | EPWM_TZ_FLAG_DCBEVT1 | EPWM_TZ_INTERRUPT);

    //
    // Acknowledge this interrupt to receive more interrupts from group 2
    //
    Interrupt_clearACKGroup(INTERRUPT_ACK_GROUP2);
}

//
// initEPWM1 - Configure ePWM1
//
void initEPWM1()
{
    //
    // Select TRIPIN1 (INPUT XBAR INPUT1) as the source for DCAH
    //
    EPWM_selectDigitalCompareTripInput(
            EPWM1_BASE, EPWM_DC_TRIP_TRIPIN1, EPWM_DC_TYPE_DCAH);

    //
    // DCAEVT1 is generated when DCAH is low
    //
    EPWM_setTripZoneDigitalCompareEventCondition(
            EPWM1_BASE, EPWM_TZ_DC_OUTPUT_A1, EPWM_TZ_EVENT_DCXH_LOW);

    //
    // DCAEVT1 uses the unfiltered version of DCAEVT1
    //
    EPWM_setDigitalCompareEventSource(
            EPWM1_BASE, EPWM_DC_MODULE_A, EPWM_DC_EVENT_1, EPWM_DC_EVENT_SOURCE_ORIG_SIGNAL);

    //
    // DCAEVT1 is asynchronous
    //
    EPWM_setDigitalCompareEventSyncMode(
            EPWM1_BASE, EPWM_DC_MODULE_A, EPWM_DC_EVENT_1, EPWM_DC_EVENT_INPUT_NOT_SYNCED);

    //
    // Select TRIPIN1 (INPUT XBAR INPUT1) as the source for DCBH
    //
    EPWM_selectDigitalCompareTripInput(
            EPWM1_BASE, EPWM_DC_TRIP_TRIPIN1, EPWM_DC_TYPE_DCBH);
    //
    // DCBEVT1 is generated when DCBH is low
    //
    EPWM_setTripZoneDigitalCompareEventCondition(
            EPWM1_BASE, EPWM_TZ_DC_OUTPUT_B1, EPWM_TZ_EVENT_DCXH_LOW);

    //
    // DCBEVT1 as input for DCFILT
    //
    EPWM_setDigitalCompareFilterInput(
            EPWM1_BASE, EPWM_DC_WINDOW_SOURCE_DCBEVT1);

    //
    // DCFILT do not invert
    //
    EPWM_disableDigitalCompareWindowInverseMode(EPWM1_BASE);

    //
    // Blanking window start at CTR=0
    //
    EPWM_setDigitalCompareBlankingEvent(EPWM1_BASE, EPWM_DC_WINDOW_START_TBCTR_ZERO);

    //
    // Blanking window period set to 6000
    //
    EPWM_setDigitalCompareWindowLength(EPWM1_BASE, 6000U);

    //
    // Blanking window starts at CTR=0 without any offset
    //
    EPWM_setDigitalCompareWindowOffset(EPWM1_BASE, 0);

    //
    // Blanking window enabled
    //
    EPWM_enableDigitalCompareBlankingWindow(EPWM1_BASE);

    //
    // DCBEVT1 is the DCFILT input
    //
    EPWM_setDigitalCompareEventSource(
            EPWM1_BASE, EPWM_DC_MODULE_B, EPWM_DC_EVENT_1, EPWM_DC_EVENT_SOURCE_FILT_SIGNAL);

    //
    // DCBEVT1 is asynchronous
    //
    EPWM_setDigitalCompareEventSyncMode(
            EPWM1_BASE, EPWM_DC_MODULE_B, EPWM_DC_EVENT_1, EPWM_DC_EVENT_INPUT_NOT_SYNCED);


    //
    // Action on DCAEVT1
    //
    EPWM_setTripZoneAction(EPWM1_BASE,
                           EPWM_TZ_ACTION_EVENT_DCAEVT1,
                           EPWM_TZ_ACTION_LOW);

    //
    // Action on DCBEVT1
    //
    EPWM_setTripZoneAction(EPWM1_BASE,
                           EPWM_TZ_ACTION_EVENT_DCBEVT1,
                           EPWM_TZ_ACTION_LOW);

    //
    // Enable TZ interrupt
    //
    EPWM_enableTripZoneInterrupt(EPWM1_BASE, EPWM_TZ_INTERRUPT_DCAEVT1);

    //
    // Set-up TBCLK
    //
    EPWM_setTimeBasePeriod(EPWM1_BASE, 12000U);
    EPWM_setPhaseShift(EPWM1_BASE, 0U);
    EPWM_setTimeBaseCounter(EPWM1_BASE, 0U);
    EPWM_setTimeBaseCounterMode(EPWM1_BASE, EPWM_COUNTER_MODE_UP_DOWN);
    EPWM_disablePhaseShiftLoad(EPWM1_BASE);

    //
    // Set ePWM clock pre-scaler
    //
    EPWM_setClockPrescaler(EPWM1_BASE,
                           EPWM_CLOCK_DIVIDER_4,
                           EPWM_HSCLOCK_DIVIDER_4);

    //
    // Set up shadowing
    //
    EPWM_setCounterCompareShadowLoadMode(EPWM1_BASE,
                                         EPWM_COUNTER_COMPARE_A,
                                         EPWM_COMP_LOAD_ON_CNTR_ZERO);

    //
    // Set-up compare
    //
    EPWM_setCounterCompareValue(EPWM1_BASE, EPWM_COUNTER_COMPARE_A, 2000U);

    //
    // Set actions
    //
    EPWM_setActionQualifierAction(EPWM1_BASE,
                                  EPWM_AQ_OUTPUT_A,
                                  EPWM_AQ_OUTPUT_HIGH,
                                  EPWM_AQ_OUTPUT_ON_TIMEBASE_UP_CMPA);
    EPWM_setActionQualifierAction(EPWM1_BASE,
                                  EPWM_AQ_OUTPUT_A,
                                  EPWM_AQ_OUTPUT_LOW,
                                  EPWM_AQ_OUTPUT_ON_TIMEBASE_DOWN_CMPA);
    EPWM_setActionQualifierAction(EPWM1_BASE,
                                  EPWM_AQ_OUTPUT_B,
                                  EPWM_AQ_OUTPUT_HIGH,
                                  EPWM_AQ_OUTPUT_ON_TIMEBASE_UP_CMPA);
    EPWM_setActionQualifierAction(EPWM1_BASE,
                                  EPWM_AQ_OUTPUT_B,
                                  EPWM_AQ_OUTPUT_LOW,
                                  EPWM_AQ_OUTPUT_ON_TIMEBASE_DOWN_CMPA);
}

//
// initTZGPIO - Configure TZ GPIO
//
void initTZGPIO(void)
{
    //
    // Set GPIO 25 as as Asynchronous input with pull up enabled
    //
    GPIO_setPadConfig(25, GPIO_PIN_TYPE_PULLUP);
    GPIO_setPinConfig(GPIO_25_GPIO25);
    GPIO_setDirectionMode(25, GPIO_DIR_MODE_IN);
    GPIO_setQualificationMode(25, GPIO_QUAL_ASYNC);

    //
    // Set GPIO 25 as TZ1 input
    //
    XBAR_setInputPin(XBAR_INPUT1, 25);

}

//
// initEPWMGPIO - Configure ePWM GPIO
//
void initEPWMGPIO(void)
{
    //
    // Disable pull up on GPIO 0 configure them as PWM1A
    //
    GPIO_setPadConfig(0, GPIO_PIN_TYPE_STD);
    GPIO_setPinConfig(GPIO_0_EPWM1A);

    //
    // Disable pull up on GPIO 0 configure them as PWM1A
    //
    GPIO_setPadConfig(1, GPIO_PIN_TYPE_STD);
    GPIO_setPinConfig(GPIO_1_EPWM1B);

}
