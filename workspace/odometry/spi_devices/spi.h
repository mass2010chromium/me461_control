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

struct SPI_request;
struct SPI;

struct SPI_request {
    uint16_t count;
    uint16_t cs;
    uint16_t* data;
    void (*isr_callback)(struct SPI*, struct SPI_request*);
    void* arg;
};
typedef struct SPI_request SPI_request;

struct SPI {
    uint16_t clock;
    uint16_t output;         // Output from F28379D
    uint16_t input;          // Input to F28379D
    uint16_t chipselect;
    volatile uint32_t write_count;
    volatile uint32_t read_count;
    volatile uint16_t recv_count;
    volatile struct SPI_REGS* regs;
    volatile uint16_t current_request;
    volatile uint16_t request_end;
    struct SPI_request requests[16];
};
typedef struct SPI SPI;

/**
 * spi: SPI struct
 * regs: SPI_RGS struct
 * clock: pin for SPICLK
 * output: pin for SPISIMO
 * input: pin for SPISOMI
 * bitrate: Bitrate (bit/s)
 */
void SPI_setup(SPI* spi, volatile struct SPI_REGS* regs, uint16_t clock, uint16_t output, uint16_t input,
                  uint32_t bitrate);

/**
 * Start spi communication.
 * Pulls out of reset, enables interrupt
 *
 * spi: SPI struct
 * wordsize: how much to transmit per write to TXBUF
 * transfer_delay: SPI clock cycles to delay after each word is written
 */
void SPI_start(SPI* spi, uint16_t wordsize, uint16_t transfer_delay);

void SPI_schedule_write(SPI* spi, uint16_t chipselect, uint16_t count, uint16_t* buf,
                        void* arg, void(*callback)(SPI*, SPI_request*));
void SPI_write(SPI* spi, uint16_t chipselect, uint16_t count, uint16_t* buf,
               void* arg, void(*callback)(SPI*, SPI_request*));

void SPI_flush(SPI* spi);

void SPI_no_callback(SPI* spi, SPI_request* request);

extern SPI spi_a;
extern SPI spi_b;
extern SPI spi_c;

#define SPI_ACK(spi) \
do { \
    volatile struct SPI_REGS* regs = (spi)->regs; \
    regs->SPIFFRX.bit.RXFFOVFCLR = 1; /* Clear Overflow flag just in case of an overflow */ \
    regs->SPIFFRX.bit.RXFFINTCLR = 1; /* Clear RX FIFO Interrupt flag so next interrupt will happen */ \
} while(0) \

__interrupt void SPIB_ISR();

#endif /* SPI_H_ */
