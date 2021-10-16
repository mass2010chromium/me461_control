/*
 * spi.c
 *
 *  Created on: Oct 13, 2021
 *      Author: jcpen
 */

#define SPI_C

#include "spi_devices/spi.h"

SPI spi_a;
SPI spi_b;
SPI spi_c;

/**
 * spi: SPI struct
 * regs: SPI_RGS struct
 * clock: pin for SPICLK
 * output: pin for SPISIMO
 * input: pin for SPISOMI
 * bitrate: Bitrate (bit/s)
 */
void SPI_setup(SPI* spi, volatile struct SPI_REGS* regs, uint16_t clock, uint16_t output, uint16_t input,
                  uint32_t bitrate, volatile PINT* isr_handle) {
    const uint32_t SPI_CLOCKRATE = 50000000;    // cycles/s
    spi->clock = clock;
    spi->output = output;
    spi->input = input;
    spi->isr_handle = isr_handle;
    EALLOW;
    GPIO_SetupPinOptions(clock, GPIO_OUTPUT, GPIO_PUSHPULL | GPIO_ASYNC | GPIO_PULLUP);
    GPIO_SetupPinOptions(output, GPIO_OUTPUT, GPIO_PUSHPULL | GPIO_ASYNC | GPIO_PULLUP);
    GPIO_SetupPinOptions(input, GPIO_INPUT, GPIO_PUSHPULL | GPIO_ASYNC | GPIO_PULLUP);
    EDIS;
    spi->regs = regs;
    regs->SPICCR.bit.SPISWRESET = 0;    // Reset before touching config
    regs->SPICTL.bit.CLK_PHASE = 1;     // @See SPI tech ref, table 18-3
    regs->SPICCR.bit.CLKPOLARITY = 0;   // This specifies "Rising Edge With Delay" (phase=1, pol=0);
    regs->SPICTL.bit.MASTER_SLAVE = 1;  // Set to SPI Master
    regs->SPICTL.bit.TALK = 1;          // Let me transmit!
    regs->SPIPRI.bit.FREE = 1;          // free run
    regs->SPICTL.bit.SPIINTENA = 0;     // Disable SPI interrupt
    regs->SPIBRR.bit.SPI_BIT_RATE = (SPI_CLOCKRATE / bitrate) - 1;
    regs->SPISTS.all = 0x0000;          // Clear status flags just in case they are set for some reason
    regs->SPIFFTX.bit.SPIRST = 1;       // Pull FIFO out of reset
    regs->SPIFFTX.bit.SPIFFENA = 1;     // Enable FIFO
    regs->SPIFFTX.bit.TXFIFO = 0;       // Write 0 to reset the FIFO pointer to zero, and hold in reset
    regs->SPIFFTX.bit.TXFFINTCLR = 1;   // Write 1 to clear SPIFFTX[TXFFINT] flag just in case it is set
    regs->SPIFFRX.bit.RXFIFORESET = 0;  // Write 0 to reset the FIFO pointer to zero, and hold in reset
    regs->SPIFFRX.bit.RXFFOVFCLR = 1;   // Write 1 to clear SPIFFRX[RXFFOVF] just in case it is set
    regs->SPIFFRX.bit.RXFFINTCLR = 1;   // Write 1 to clear SPIFFRX[RXFFINT] flag just in case it is set
    regs->SPIFFRX.bit.RXFFIENA = 1;     // Enable the RX FIFO Interrupt. RXFFST >= RXFFIL
}

/**
 * Start spi communication.
 * Pulls out of reset, enables interrupt
 *
 * spi: SPI struct
 * wordsize: how much to transmit per write to TXBUF
 * transfer_delay: SPI clock cycles to delay after each word is written
 */
void SPI_start(SPI* spi, uint16_t wordsize, uint16_t transfer_delay, PINT callback) {

    EALLOW;
    *spi->isr_handle = callback;
    EDIS;

    volatile struct SPI_REGS* regs = spi->regs;
    regs->SPICCR.bit.SPICHAR = wordsize-1;
    regs->SPIFFCT.bit.TXDLY = transfer_delay;
    regs->SPICCR.bit.SPISWRESET = 1;    // Pull the SPI out of reset
    regs->SPIFFTX.bit.TXFIFO = 1;       // Release transmit FIFO from reset.
    regs->SPIFFRX.bit.RXFIFORESET = 1;  // Re-enable receive FIFO operation
    regs->SPICTL.bit.SPIINTENA = 1;     // Enables SPI interrupt. !! I don’t think this is needed. Need to Test
    regs->SPIFFRX.bit.RXFFIL = 0x10;    // Interrupt Level to 16 words or more received into FIFO causes
                                        // interrupt. This is just the initial setting for the register. Will be changed later
}

/**
 * Write some words to the SPI channel.
 *
 * spi : SPI struct
 * chipselect : Pin to use as chipselect (active low, TODO: parametrize)
 * count : Num words to write
 * buf : Words to write
 * callback : Callback function for the interrupt
 * arg : Single argument that gets passed to the callback function
 */
void SPI_write(SPI* spi, uint16_t chipselect, uint16_t count, uint16_t* buf, void* arg) {
    volatile struct SPI_REGS* regs = spi->regs;
    spi->recv_count = 0;
    spi->chipselect = chipselect;

    spi->arg = arg;
    spi->write_count++;
    setGPIO(chipselect, 0);             // TODO: parametrize
    regs->SPIFFRX.bit.RXFFIL = count; // Issue the SPIB_RX_INT when two values are in the RX FIFO
    uint16_t i;
    for (i = 0; i < count; ++i) {
        regs->SPITXBUF = buf[i];
    }
}

__interrupt void SPI_no_callback() {};
