/*
 * gpio_decl.h
 *
 * GPIO utility functions and macros.
 *
 *  Created on: Oct 13, 2021
 *      Author: jcpen
 */

#ifndef GPIO_DECL_H_
#define GPIO_DECL_H_

#include <stdio.h>
#include <stdlib.h>
#include <stdarg.h>
#include <string.h>
#include <math.h>
#include <limits.h>
#include "F28x_Project.h"

/*
 * Macros for setting and getting GPIO pins by group and number.
 * Use these only if speed is an absolute must.
 */
#ifdef GPIO_SET_NODAT
#define GPIO_SET(x, n, a) if(a) {GPIO_ON(x, n);} else {GPIO_OFF(x, n);}
#else
#define GPIO_SET(x, n, a) GpioDataRegs.GP ## x ## DAT.bit.GPIO ## n = (a)
#endif
#define GPIO_ON(x, n) (GpioDataRegs.GP ## x ## SET.bit.GPIO ## n = 1)
#define GPIO_OFF(x, n) (GpioDataRegs.GP ## x ## CLEAR.bit.GPIO ## n = 1)
#define GPIO_GET(x, n) (GpioDataRegs.GP ## x ## DAT.bit.GPIO ## n)

/*
 * One "slice" of GpioDataRegs. Used for indexing the struct as an array.
 */
typedef struct {
    uint32_t CTRL;                      // GPIO A Qualification Sampling Period Control (GPIO0 to 31)
    uint32_t QSEL1;                     // GPIO A Qualifier Select 1 Register (GPIO0 to 15)
    uint32_t QSEL2;                     // GPIO A Qualifier Select 2 Register (GPIO16 to 31)
    uint32_t MUX1;                      // GPIO A Mux 1 Register (GPIO0 to 15)
    uint32_t MUX2;                      // GPIO A Mux 2 Register (GPIO16 to 31)
    uint32_t DIR;                       // GPIO A Direction Register (GPIO0 to 31)
    uint32_t PUD;                       // GPIO A Pull Up Disable Register (GPIO0 to 31)
    uint16_t rsvd1[2];                  // Reserved
    uint32_t INV;                       // GPIO A Input Polarity Invert Registers (GPIO0 to 31)
    uint32_t ODR;                       // GPIO A Open Drain Output Register (GPIO0 to GPIO31)
    uint16_t rsvd2[12];                 // Reserved
    uint32_t GMUX1;                     // GPIO A Peripheral Group Mux (GPIO0 to 15)
    uint32_t GMUX2;                     // GPIO A Peripheral Group Mux (GPIO16 to 31)
    uint16_t rsvd3[4];                  // Reserved
    uint32_t CSEL1;                     // GPIO A Core Select Register (GPIO0 to 7)
    uint32_t CSEL2;                     // GPIO A Core Select Register (GPIO8 to 15)
    uint32_t CSEL3;                     // GPIO A Core Select Register (GPIO16 to 23)
    uint32_t CSEL4;                     // GPIO A Core Select Register (GPIO24 to 31)
    uint16_t rsvd4[12];                 // Reserved
    uint32_t LOCK;                      // GPIO A Lock Configuration Register (GPIO0 to 31)
    uint32_t CR;                        // GPIO A Lock Commit Register (GPIO0 to 31)
} GPIOControlBlock;

typedef struct {
    uint32_t DAT;                       // GPIO A Data Register (GPIO0 to 31)
    uint32_t SET;                       // GPIO A Data Set Register (GPIO0 to 31)
    uint32_t CLEAR;                     // GPIO A Data Clear Register (GPIO0 to 31)
    uint32_t TOGGLE;                    // GPIO A Data Toggle Register (GPIO0 to 31)
} GPIODataBlock;

/**
 * Set a GPIO pin by number only.
 * Bit should be 0 or 1.
 * Pin should be a number.
 * More expensive than GPIO_SET macro.
 *
 * Parameters:
 *      pin: Pin to set (by number)
 *      bit: zero to clear, nonzero to set
 */
inline void setGPIO(int16_t pin, int16_t bit) {

    volatile GPIODataBlock* gpioBaseAddr = (GPIODataBlock*) &GpioDataRegs;
    volatile GPIODataBlock* root = &(gpioBaseAddr[(pin / 32)]);
    int16_t index = pin & 0x1f;
    if (bit) {
        // GPXSET.all
        root->SET |= ((uint32_t)1) << index;
    }
    else {
        root->CLEAR |= ((uint32_t)1) << index;
    }
}

/**
 * Condensed SetupPinOptions and SetupPinMux.
 * Hardcoded to CPU1 right now but should be easy to change with macro.
 *
 * Parameters:
 *      pin: Pin to set (by number)
 *      mux: pinmux setting. {@see GPIO_SetupPinMux}
 *      type: GPIO_INPUT or GPIO_OUTPUT
 *      flags: Additional flags (GPIO_PUSHPULL for ex)
 */
inline void setupGPIO(int16_t pin, int16_t mux, int16_t type, int16_t flags) {
    GPIO_SetupPinMux(pin, GPIO_MUX_CPU1, mux);
    GPIO_SetupPinOptions(pin, type, flags);
}
/**
 * setupGPIO but with initial value.
 * Parameters:
 *      pin: Pin to set (by number)
 *      mux: pinmux setting. {@see GPIO_SetupPinMux}
 *      type: GPIO_INPUT or GPIO_OUTPUT
 *      flags: Additional flags (GPIO_PUSHPULL for ex)
 *      init: Initial value for the pin
 */
inline void initGPIO(int16_t n, int16_t mux, int16_t type, int16_t flags, int16_t init) {
    setupGPIO(n, mux, type, flags);
    setGPIO(n, init);
}
#endif /* GPIO_DECL_H_ */
