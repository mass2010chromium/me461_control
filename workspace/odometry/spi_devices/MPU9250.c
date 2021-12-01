/*
 * MPU9250.c
 *
 *  Created on: Oct 13, 2021
 *      Author: jcpen
 */

#include "spi_devices/MPU9250.h"
#include "filter.h"

void MPU9250_set_scaling(MPU9250* imu, float dps, float gs) {
    imu->gyro_scaling = ((float) dps) / 32768;
    imu->accel_scaling = ((float) gs) / 32768;
}

inline void __MPU9250_read_SPI(volatile struct SPI_REGS* regs, volatile uint16_t* write_head, uint16_t packet_size) {
    uint16_t i;
    uint16_t read_data = regs->SPIRXBUF;
    *(write_head++) = read_data & 0xff;
    for (i = 1; i < packet_size; ++i) {
        read_data = regs->SPIRXBUF;
        *(write_head++) = read_data >> 8;
        *(write_head++) = read_data & 0xff;
    }
}

void __MPU9250_recv(SPI* spi, SPI_request* request) {
    MPU9250* _imu = (MPU9250*) request->arg;
    volatile struct SPI_REGS* regs = spi->regs;

    uint16_t packet_size = *_imu->packet_sizes_head;
    if (_imu->packets_head[0] & 0x8000) {    // READ operation
        uint16_t* write_head = &_imu->registers[((*_imu->packets_head) >> 8) & 0x7F];
        __MPU9250_read_SPI(regs, write_head, packet_size);
    }
    else {
        while (packet_size--) {
            uint16_t unused = regs->SPIRXBUF;
        }
    }
    ++_imu->packet_sizes_head;
    if (*_imu->packet_sizes_head == 0) {
        _imu->status = MPU9250_IDLE;

        return;
    }
    _imu->packets_head += packet_size;
    MPU9250_write_async(_imu);
}

void MPU9250_write_async(MPU9250* imu) {
    SPI* spi = imu->channel;
    uint16_t packet_size = *imu->packet_sizes_head;

    SPI_schedule_write(spi, imu->chipselect, packet_size, imu->packets_head, imu, __MPU9250_recv);
}

void MPU9250_write_blocking(MPU9250* imu) {
    SPI* spi = imu->channel;
    volatile struct SPI_REGS* regs = spi->regs;

    volatile uint16_t* packets_head = imu->packets;
    volatile uint16_t* packet_sizes = imu->packet_sizes;
    for (; *packet_sizes; ++packet_sizes) {
        uint16_t packet_size = *packet_sizes;

        SPI_write(spi, imu->chipselect, packet_size, packets_head, 0, SPI_no_callback);
        spi->request_end = 0;
        while(regs->SPIFFRX.bit.RXFFST != packet_size);

        setGPIO(spi->chipselect, 1);        // TODO: parametrize
        if ((*packets_head) & 0x8000) {    // READ operation
            uint16_t* write_head = &imu->registers[((*packets_head) >> 8) & 0x7F];
            __MPU9250_read_SPI(regs, write_head, packet_size);
        }
        else {
            while (packet_size--) {
                uint16_t unused = regs->SPIRXBUF;
            }
        }
        ++spi->read_count;
        packets_head += packet_size;
        DELAY_US(10);
    }
    DELAY_US(40);

    // Clear SPI interrupt source just in case it was issued.
    regs->SPIFFRX.bit.RXFFOVFCLR=1; // Clear Overflow flag
    regs->SPIFFRX.bit.RXFFINTCLR=1; // Clear Interrupt flag
    PieCtrlRegs.PIEACK.all = PIEACK_GROUP6;     // Shared by all 3 SPI interfaces

    imu->status = MPU9250_IDLE;
}

void MPU9250_update_stats(MPU9250* imu) {
    imu->accel_x = ((int) (imu->registers[0x3B] << 8 | imu->registers[0x3C])) * imu->accel_scaling;
    imu->accel_y = ((int) (imu->registers[0x3D] << 8 | imu->registers[0x3E])) * imu->accel_scaling;
    imu->accel_z = ((int) (imu->registers[0x3F] << 8 | imu->registers[0x40])) * imu->accel_scaling;

    imu->gyro_x = (((int) (imu->registers[0x43] << 8 | imu->registers[0x44])) - imu->gyro_off_x) * imu->gyro_scaling;
    imu->gyro_y = (((int) (imu->registers[0x45] << 8 | imu->registers[0x46])) - imu->gyro_off_y) * imu->gyro_scaling;
    imu->gyro_z = (((int) (imu->registers[0x47] << 8 | imu->registers[0x48])) - imu->gyro_off_z) * imu->gyro_scaling;
}

void MPU9250_set_read_accel_gyro(MPU9250* imu) {
    imu->packet_sizes[0] = 8;
    imu->packet_sizes[1] = 0;
    imu->packets[0] = 0x3B00 | 0x8000;
    imu->packet_sizes_head = imu->packet_sizes;
    imu->packets_head = imu->packets;
    uint16_t i;
    for (i = 1; i < 8; ++i) {
        imu->packets[i] = 0;
    }

    imu->status = MPU9250_ACTIVE;

    if (imu->IO_mode == MPU9250_ASYNC) {
        SPI_schedule_write(imu->channel, imu->chipselect, 8, imu->packets_head, imu, __MPU9250_recv);
    }
    else {
        MPU9250_write_blocking(imu);
    }
}

void MPU9250_calibrate(MPU9250* imu, uint16_t iterations) {
    uint16_t counter = iterations;
    int32_t gyro_x_accum = 0;
    int32_t gyro_y_accum = 0;
    int32_t gyro_z_accum = 0;

    while (counter--) {
        MPU9250_set_read_accel_gyro(imu);

        while (imu->status != MPU9250_IDLE);

        gyro_x_accum += (int) (imu->registers[0x43] << 8 | imu->registers[0x44]);
        gyro_y_accum += (int) (imu->registers[0x45] << 8 | imu->registers[0x46]);
        gyro_z_accum += (int) (imu->registers[0x47] << 8 | imu->registers[0x48]);
    }
    imu->gyro_off_x = gyro_x_accum / iterations;
    imu->gyro_off_y = gyro_y_accum / iterations;
    imu->gyro_off_z = gyro_z_accum / iterations;
    imu->calibrated = 1;
}

void MPU9250_flush(MPU9250* imu) {
    imu->packet_sizes_head = imu->packet_sizes;
    imu->packets_head = imu->packets;
    uint16_t cur_type = imu->flags[0];
    uint16_t i = 0;
    for (; i < MPU9250_NUMREGS;) {
        if (cur_type) {
            imu->flags[i] = 0;
            uint16_t packet_size = 1;
            *(imu->packets_head++) = (cur_type << 15) | (i << 8) | imu->registers[i];

            while (1) {
                ++i;
                uint16_t next = imu->flags[i];
                if (next) {
                    if (next == cur_type) {
                        uint16_t next2 = imu->flags[i+1];
                        if (next2 == cur_type) {
                            *(imu->packets_head++) = (imu->registers[i] << 8) | imu->registers[i+1];
                            ++packet_size;
                            imu->flags[i] = 0;
                            ++i;
                            imu->flags[i] = 0;
                            if (packet_size == 16) {
                                // Maximum packet size.
                                break;
                            }
                            continue;
                        }
                    }
                    cur_type = next;
                    break;
                }
                ++i;
                cur_type = imu->flags[i];
                break;
            }
            *(imu->packet_sizes_head++) = packet_size;
        }
        else {
            ++i;
            cur_type = imu->flags[i];
        }
    }
    *imu->packet_sizes_head = 0;
    imu->packet_sizes_head = imu->packet_sizes;
    imu->packets_head = imu->packets;

    if (*imu->packet_sizes_head == 0) {
        imu->status = MPU9250_IDLE;
        return;
    }

    imu->status = MPU9250_ACTIVE;

    if (imu->IO_mode == MPU9250_ASYNC) {
        MPU9250_write_async(imu);
    }
    else {
        MPU9250_write_blocking(imu);
    }
}
