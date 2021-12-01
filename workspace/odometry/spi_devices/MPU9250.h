/*
 * MPU9250.h
 *
 *  Created on: Oct 13, 2021
 *      Author: jcpen
 */

#ifndef MPU9250_H_
#define MPU9250_H_

#include <stdio.h>
#include <stdlib.h>
#include <stdarg.h>
#include <string.h>
#include <math.h>
#include <limits.h>
#include "F28x_Project.h"
#include "spi_devices/spi.h"
#include "spi_devices/manual_spi.h"

#define MPU9250_WRITE 2
#define MPU9250_READ 1
#define MPU9250_NONE 0
#define MPU9250_ACTIVE 1
#define MPU9250_IDLE 0
#define MPU9250_ASYNC 0
#define MPU9250_BLOCKING 1
#define MPU9250_NUMREGS 128

typedef struct {
    SPI* channel;
    volatile uint16_t chipselect;
    volatile uint16_t status;    // 0 for idle, 1 for communicating
    volatile uint16_t calibrated;
    volatile uint16_t IO_mode;
    float accel_scaling;
    float gyro_scaling;
    float accel_x;
    float accel_y;
    float accel_z;
    float gyro_x;
    float gyro_y;
    float gyro_z;
    int16_t gyro_off_x;
    int16_t gyro_off_y;
    int16_t gyro_off_z;
    uint16_t registers[MPU9250_NUMREGS];
    uint16_t flags[MPU9250_NUMREGS+2];
    volatile uint16_t* packet_sizes_head;
    volatile uint16_t* packets_head;
    volatile uint16_t packet_sizes[MPU9250_NUMREGS+2];
    volatile uint16_t packets[MPU9250_NUMREGS+2];
} MPU9250;

void MPU9250_set_scaling(MPU9250* imu, float dps, float gs);

__interrupt void MPU9250_SPIB_ISR ();

void MPU9250_write_async(MPU9250* imu);

void MPU9250_write_blocking(MPU9250* imu);

void MPU9250_update_stats(MPU9250* imu);

void MPU9250_set_read_accel_gyro(MPU9250* imu);

void MPU9250_calibrate(MPU9250* imu, uint16_t iterations);

void MPU9250_flush(MPU9250* imu);

inline void MPU9250_clear_flags(MPU9250* imu) {
    memset(imu->flags, 0, sizeof(imu->flags) / sizeof(imu->flags[0]));
}

inline void MPU9250_set_write(MPU9250* imu, uint16_t reg, uint16_t value) {
    imu->registers[reg] = value;
    imu->flags[reg] = MPU9250_WRITE;
}

inline void MPU9250_set_read(MPU9250* imu, uint16_t reg) {
    imu->registers[reg] = 0;
    imu->flags[reg] = MPU9250_READ;
}

#endif /* MPU9250_H_ */
