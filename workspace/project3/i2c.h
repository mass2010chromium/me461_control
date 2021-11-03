/*
 * i2c.h
 *
 * Helper functions for I2C communication.
 *
 *  Created on: Oct 29, 2021
 *      Author: jcpen
 */

#ifndef I2C_H_
#define I2C_H_

#include "F28x_Project.h"
#include "gpio_decl.h"

/**
 * Initialize an I2C register set with some default parameters (given in class).
 * TODO: make everything configurable
 */
void I2C_init(volatile struct I2C_REGS* i2c);

/**
 * Write multiple words (bytes) out to I2C.
 *
 * Parameters:
 *      i2c: pointer to the I2C struct representing the I2C module to write to.
 *      address: Address of the I2C device to write to.
 *      reg: Register to start writing to.
 *      n: Number of words to send.
 *      data: Array of words to transmit.
 *
 * Return:
 *      0 on success
 *      2 on I2C busy
 *      3 on NACK
 */
uint16_t write_to_I2C(volatile struct I2C_REGS* i2c, uint16_t address, uint16_t reg, uint16_t n, uint16_t* data);

/**
 * Read multiple words (bytes) from I2C.
 *
 * Parameters:
 *      i2c: pointer to the I2C struct representing the I2C module to read from.
 *      address: Address of the I2C device to read from.
 *      reg: Register to start reading from.
 *      n: Number of words to read.
 *      data: Array that read words will be placed into..
 *
 * Return:
 *      0 on success
 *      2 on I2C busy
 *      3 on NACK
 */
uint16_t read_from_I2C(volatile struct I2C_REGS* i2c, uint16_t address, uint16_t reg, uint16_t n, uint16_t* data);

#endif /* I2C_H_ */
