/*
 * spi.h
 *
 *  Created on: Oct 13, 2021
 *      Author: jcpen
 */

#ifndef SPI_H_
#define SPI_H_

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
#include "gpio_decl.h"

typedef struct {
    uint16_t clock;
    uint16_t output;         // Output from F28379D
    uint16_t input;          // Input to F28379D
    uint16_t chipselect;
    uint32_t write_count;
    uint32_t read_count;
    uint16_t recv_count;
    volatile PINT* isr_handle;
    void* arg;
    volatile struct SPI_REGS* regs;
} SPI;

/**
 * spi: SPI struct
 * regs: SPI_RGS struct
 * clock: pin for SPICLK
 * output: pin for SPISIMO
 * input: pin for SPISOMI
 * bitrate: Bitrate (bit/s)
 */
void SPI_setup(SPI* spi, volatile struct SPI_REGS* regs, uint16_t clock, uint16_t output, uint16_t input,
                  uint32_t bitrate, volatile PINT* isr_handle);

/**
 * Start spi communication.
 * Pulls out of reset, enables interrupt
 *
 * spi: SPI struct
 * wordsize: how much to transmit per write to TXBUF
 * transfer_delay: SPI clock cycles to delay after each word is written
 */
void SPI_start(SPI* spi, uint16_t wordsize, uint16_t transfer_delay, PINT callback);

void SPI_write(SPI* spi, uint16_t chipselect, uint16_t count, uint16_t* buf, void* arg);

__interrupt void SPI_no_callback();

extern SPI spi_a;
extern SPI spi_b;
extern SPI spi_c;

#define SPI_ACK(spi) \
do { \
    volatile struct SPI_REGS* regs = (spi)->regs; \
    regs->SPIFFRX.bit.RXFFOVFCLR = 1; /* Clear Overflow flag just in case of an overflow */ \
    regs->SPIFFRX.bit.RXFFINTCLR = 1; /* Clear RX FIFO Interrupt flag so next interrupt will happen */ \
} while(0) \

#define SPI_ISR(name, spi_struct, callback) \
__interrupt void name () { \
    SPI* spi = &(spi_struct); \
    volatile struct SPI_REGS* regs = spi->regs; \
    uint16_t num_recv = regs->SPIFFRX.bit.RXFFST; \
    setGPIO(spi->chipselect, 1);        /* TODO: parametrize */ \
    spi->recv_count = num_recv; \
    ++spi->read_count; \
\
    callback(spi, spi->arg); \
    PieCtrlRegs.PIEACK.all = PIEACK_GROUP6; /* Acknowledge INT6 PIE interrupt */ \
}

#endif /* SPI_H_ */
