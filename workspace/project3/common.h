/*
 * common.h
 *
 *  Created on: Oct 1, 2021
 *      Author: jcpen
 */

#ifndef COMMON_H_
#define COMMON_H_
#include <stdio.h>
#include <stdlib.h>
#include <stdarg.h>
#include <string.h>
#include <math.h>
#include <limits.h>
#include "F28x_Project.h"

#include "pwm.h"
#include "adc_dac.h"
#include "filter.h"
#include "spi_devices/spi.h"
#include "spi_devices/MPU9250.h"
#include "gpio_decl.h"
#include "i2c.h"

#define PI          3.1415926535897932384626433832795
#define TWOPI       6.283185307179586476925286766559
#define HALFPI      1.5707963267948966192313216916398


#define DAN_ADC_TO_VOLTS (3.3 / 4095.0)

/**
 * 16 display LEDs by GPIO pin number, in column major order (except number 15 which is the odd one out).
 */
const int16_t DISPLAY_LEDS[16] = {22, 94, 95, 97, 111, 130, 131, 25, 26, 27, 60, 61, 157, 158, 159, 160};

/**
 * Lowest 5 significant bits of the input are used to set the state of the 5 LED rows.
 */
void SetLEDRowsOnOff(int16_t rows) {

    // ROW 1 Off Use SET.bit for On and TOGGLE.bit fto toggle On/Off
    int16_t i;
    rows = rows & 0x1f;
    for (i = 0; i < 5; ++i) {
        int16_t bit = rows & 0x1;
        rows = rows >> 1;
        setGPIO(DISPLAY_LEDS[i], bit);
        setGPIO(DISPLAY_LEDS[i+5], bit);
        setGPIO(DISPLAY_LEDS[i+10], bit);
    }
}

/**
 * Read state of the four pushbuttons, in bit-packed form.
 */
int16_t ReadSwitches() {
    return GPIO_GET(A, 4) | (GPIO_GET(A, 5)<<1) | (GPIO_GET(A, 6)<<2) | (GPIO_GET(A, 7)<<3);
}

/**
 * Parse two bytes from a buffer in (lsb, msb) into a single integer.
 * Good for unpacking stuff like from I2C
 *
 * Parameters:
 *      dat: input buffer
 *
 * Return:
 *      The parse result
 */
inline uint16_t lsbmsb_parse(uint16_t* dat) {
    return (dat[1] << 8) | ((dat[0]) & 0xff);
}

/**
 * "Pack" a 16 bit number into (lsb, msb) order in a buffer.
 * Good for transmitting over stuff like I2C
 *
 * Parameters:
 *      out: output buffer
 *      in: number to convert
 */
inline void lsbmsb_convert(uint16_t* out, uint16_t in) {
    out[0] = in & 0xff;
    out[1] = in >> 8;
}

/*
 * Convert a numeric string to decimal. No sign support.
 *
 * Parameters:
 *      str: Pointer to the start of the string to parse
 *
 * Return:
 *      Decimal number made by grabbing as many digits from the string
 *      as possible (until a non [0-9] char is encountered, including EOL).
 */
inline uint16_t parse_num(const char* str) {
    uint16_t res = 0;
    for (;; ++str) {
        uint16_t num = *str - '0';
        if (num > 9) break;
        res = res*10 + num;
    }
    return res;
}

/*
 * Convert a numeric string to hex. No sign support.
 *
 * Parameters:
 *      str: Pointer to the start of the string to parse
 *
 * Return:
 *      Hex number made by grabbing as many digits from the string
 *      as possible (until a non [0-9A-F] char is encountered,
 *      including EOL).
 *
 *      NOTE: Uppercase A-F only!
 */
uint16_t parse_hex(const char* str) {
    uint16_t res = 0;
    for (;; ++str) {
        uint16_t num = *str - '0';
        if (num > 9) {
            num = *str - 'A';
            if (num > 5) break;
            num += 10;
        }
        res = (res << 4) + num;
    }
    return res;
}

/**
 * Converts a single byte to bcd. Max value 99
 *
 * Parameters:
 *      in: input number (decimal, less than 99 or UB)
 *
 * Return:
 *      BCD encoded number (two bytes)
 */
uint16_t to_bcd(uint16_t in) {
    if (in >= 100) return 0;
    static uint16_t lookup[100] = {  0,   1,   2,   3,   4,   5,   6,   7,   8,   9,
                                    16,  17,  18,  19,  20,  21,  22,  23,  24,  25,
                                    32,  33,  34,  35,  36,  37,  38,  39,  40,  41,
                                    48,  49,  50,  51,  52,  53,  54,  55,  56,  57,
                                    64,  65,  66,  67,  68,  69,  70,  71,  72,  73,
                                    80,  81,  82,  83,  84,  85,  86,  87,  88,  89,
                                    96,  97,  98,  99, 100, 101, 102, 103, 104, 105,
                                   112, 113, 114, 115, 116, 117, 118, 119, 120, 121,
                                   128, 129, 130, 131, 132, 133, 134, 135, 136, 137,
                                   144, 145, 146, 147, 148, 149, 150, 151, 152, 153};
    return lookup[in];
}

/**
 * Convert a BCD number to normal binary encoding.
 *
 * Parameters:
 *      in: Number in BCD form
 *
 * Return:
 *      Converted to normal binary encoding.
 */
uint16_t from_bcd(uint16_t in) {
    uint16_t upper = (in & 0xf0);
    //   (lower bits) + (upper * 8)  + (upper * 2)
    return (in & 0xf) + (upper >> 1) + (upper >> 3);
}

#endif /* COMMON_H_ */
