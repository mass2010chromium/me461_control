
#ifndef TEST_DEVICE_H
#define TEST_DEVICE_H

#include <stdio.h>
#include <stdlib.h>
#include <stdarg.h>
#include <string.h>
#include <math.h>
#include <limits.h>
#include "F28x_Project.h"
#include "spi_devices/spi.h"

inline void __manual_SPI_ISR(SPI* spi, uint16_t* buf) {
    uint16_t i;
    if (buf) {
        for (i = 0; i < spi->recv_count; ++i) {
            buf[i] = spi->regs->SPIRXBUF;
        }
    }
    else {
        for (i = 0; i < spi->recv_count; ++i) {
            uint16_t unused = spi->regs->SPIRXBUF;
        }
    }
}

__interrupt void manual_SPIB_ISR();

#endif // TEST_DEVICE_H
