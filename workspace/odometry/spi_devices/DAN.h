/*
 * DAN.h
 *
 *  Created on: Nov 20, 2021
 *      Author: jcpen
 */

#ifndef SPI_DEVICES_DAN_H_
#define SPI_DEVICES_DAN_H_
#include "spi.h"

#define DAN_ADC_TO_VOLTS (3.3 / 4095.0)

typedef struct {
    int chipselect;
    SPI* spi;
    volatile float ADC1;
    volatile float ADC2;
    float command1;
    float command2;
    uint16_t buf[3];
} DAN;

extern DAN dan;

void DAN_init(DAN* this, int chipselect, SPI* spi);

void DAN_set_write(DAN* this, float command1, float command2);

#endif /* SPI_DEVICES_DAN_H_ */
