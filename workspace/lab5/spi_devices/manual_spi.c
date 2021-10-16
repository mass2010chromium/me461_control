/*
 * manual_spi.c
 *
 *  Created on: Oct 15, 2021
 *      Author: jcpen
 */

#include "spi_devices/manual_spi.h"

SPI_ISR(manual_SPIB_ISR, spi_b, __manual_SPI_ISR);
