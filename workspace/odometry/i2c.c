/*
 * i2c.c
 *
 * Helper functions for I2C communication.
 *
 *  Created on: Oct 29, 2021
 *      Author: jcpen
 */

#include "i2c.h"

/**
 * Initialize an I2C register set with some default parameters (given in class).
 * TODO: make everything configurable
 */
void I2C_init(volatile struct I2C_REGS* i2c)
{
    EALLOW;
    /* Enable internal pull-up for the selected I2C pins */
    GpioCtrlRegs.GPBPUD.bit.GPIO40 = 1;
    GpioCtrlRegs.GPBPUD.bit.GPIO41 = 1;
    /* Set qualification for the selected I2C pins */
    GpioCtrlRegs.GPBQSEL1.bit.GPIO40 = 3;
    GpioCtrlRegs.GPBQSEL1.bit.GPIO41 = 3;
    /* Configure which of the possible GPIO pins will be I2C_B pins using GPIO regs*/
    GpioCtrlRegs.GPBGMUX1.bit.GPIO40 = 1;
    GpioCtrlRegs.GPBMUX1.bit.GPIO40 = 2;
    GpioCtrlRegs.GPBGMUX1.bit.GPIO41 = 1;
    GpioCtrlRegs.GPBMUX1.bit.GPIO41 = 2;

//    setupGPIO(40, 6, GPIO_OUTPUT, GPIO_PULLUP | GPIO_ASYNC);
//    setupGPIO(41, 6, GPIO_OUTPUT, GPIO_PULLUP | GPIO_ASYNC);

    EDIS;
    // Initialize I2C
    i2c->I2CMDR.bit.IRS = 0;
    // 200MHz / 20 = 10MHz
    i2c->I2CPSC.all = 19;
    // 10MHz/40 = 250KHz
    i2c->I2CCLKL = 15; //psc > 2 so d = 5 See Usersguide
    i2c->I2CCLKH = 15; //psc > 2 so d = 5 See Usersguide
    i2c->I2CIER.all = 0x00;
    i2c->I2CMDR.bit.IRS = 1;
    DELAY_US(5000);
}

/**
 * Write a single word (8 bits) out to I2C.
 *
 * Parameters:
 *      i2c: pointer to the I2C struct representing the I2C module to write to.
 *      data: word to transmit.
 *
 * Return:
 *      0 on success, 3 on NACK
 */
uint16_t write_single_I2C(volatile struct I2C_REGS* i2c, uint16_t data) {
    while(!i2c->I2CSTR.bit.XRDY);   //Poll until I2C ready to Transmit
    i2c->I2CDXR.all = data;         // Write Command 1 LSB
    if (i2c->I2CSTR.bit.NACK == 1){ // Check for No Acknowledgment
        return 3;                   // This should not happen
    }
    return 0;
}

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
uint16_t write_to_I2C(volatile struct I2C_REGS* i2c, uint16_t address, uint16_t reg, uint16_t n, uint16_t* data) {
    DELAY_US(2000);  // Allow time for I2C to finish up previous commands. It pains me to have this
                    // delay here but I have not had time to figure out what status bit to poll on to
                    // to check if the I2C peripheral is ready of the next command. I have tried the busy
                    // bit but that gave me some issues especially at startup. This would be a great
                    // choice for a part of your final project if you would like to learn more about I2C.
    if (i2c->I2CSTR.bit.BB == 1) {  // Check if I2C busy, if it is better
        return 2;                   // to Exit and try again next sample
    }                               // This should not happen too often
    while(!i2c->I2CSTR.bit.XRDY);   //Poll until I2C ready to Transmit
    i2c->I2CSAR.all = address;      // I2C address of ChipXYZ
    i2c->I2CCNT = n + 1;            //Num Values plus Start Register n + 1
    i2c->I2CMDR.all = 0x6E20;       // I2C in master mode (MST), I2C is in transmit mode (TRX) with start and stop
    uint16_t i, res;
    res = write_single_I2C(i2c, reg); // First write the Register value
    if (res) return res;
    for (i = 0; i < n; ++i) {
        res = write_single_I2C(i2c, data[i]);
        if (res) return res;
    }
    return 0;
}

/**
 * Read a single word (8 bits) from I2C.
 *
 * Parameters:
 *      i2c: pointer to the I2C struct representing the I2C module to read from.
 *      data: pointer to place to put the received word.
 *
 * Return:
 *      0 on success, 3 on NACK
 */
uint16_t read_single_I2C(volatile struct I2C_REGS* i2c, uint16_t* data) {
    while(!i2c->I2CSTR.bit.RRDY);   //Poll until I2C ready to Transmit
    *data = i2c->I2CDRR.all;        // Write Command 1 LSB
    if (i2c->I2CSTR.bit.NACK == 1){ // Check for No Acknowledgment
        return 3;                   // This should not happen
    }
    return 0;
}


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
uint16_t read_from_I2C(volatile struct I2C_REGS* i2c, uint16_t address, uint16_t reg, uint16_t n, uint16_t* data) {
    DELAY_US(2000);  // Allow time for I2C to finish up previous commands. It pains me to have this
                    // delay here but I have not had time to figure out what status bit to poll on to
                    // to check if the I2C peripheral is ready of the next command. I have tried the busy
                    // bit but that gave me some issues especially at startup. This would be a great
                    // choice for a part of your final project if you would like to learn more about I2C.
    if (i2c->I2CSTR.bit.BB == 1) {  // Check if I2C busy, if it is better
        return 2;                   // to Exit and try again next sample
    }                               // This should not happen too often
    while(!i2c->I2CSTR.bit.XRDY);   //Poll until I2C ready to Transmit
    i2c->I2CSAR.all = address;      // I2C address of ChipXYZ
    i2c->I2CCNT = 1;                // Register
    i2c->I2CMDR.all = 0x6620;       // I2C in master mode (MST), I2C is in transmit mode (TRX) with start, no stop
    uint16_t i, res;
    res = write_single_I2C(i2c, reg);   // Use write mode to set address
    if (res) return res;

    while(!i2c->I2CSTR.bit.XRDY);   //Poll until I2C ready to Transmit

    i2c->I2CSAR.all = address;      // I2C address of ChipXYZ
    i2c->I2CCNT = n;                // n to read
    i2c->I2CMDR.all = 0x6C20;       // I2C in master mode (MST), I2C is in receive mode with start and stop
    if (I2cbRegs.I2CSTR.bit.NACK == 1) {
        return 3;
    }
    for (i = 0; i < n; ++i) {
        res = read_single_I2C(i2c, &data[i]);
        if (res) {
            return res;
        }
    }
    return 0;
}
