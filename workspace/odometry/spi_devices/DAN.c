/*
 * DAN.c
 *
 *  Created on: Nov 20, 2021
 *      Author: jcpen
 */

#include "DAN.h"

DAN dan;

void DAN_init(DAN* this, int chipselect, SPI* spi) {
    this->chipselect = chipselect;
    this->spi = spi;
    this->buf[0] = 0x00DA;
}

void DAN_int(SPI* spi, SPI_request* request) {
    DAN* this = (DAN*) request->arg;
    uint16_t recv = spi->regs->SPIRXBUF;
    recv = spi->regs->SPIRXBUF;
    this->ADC1 = DAN_ADC_TO_VOLTS * recv;
    recv = spi->regs->SPIRXBUF;
    this->ADC2 = DAN_ADC_TO_VOLTS * recv;
}

void DAN_set_write(DAN* this, float command1, float command2) {
    this->buf[1] = 1500 + ((int)(command1 * 1500));
    this->buf[2] = 1500 + ((int)(command2 * 1500));
    SPI_schedule_write(this->spi, this->chipselect, 3, this->buf, this, DAN_int);
}


