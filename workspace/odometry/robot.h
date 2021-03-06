/*
 * segbot.h
 *
 *  Created on: Nov 20, 2021
 *      Author: jcpen
 */
#pragma once

#include "filter.h"
#include "spi_devices/spi.h"
#include "spi_devices/MPU9250.h"
#include "spi_devices/DAN.h"
#include "gpio_decl.h"
#include "eqep.h"
#include "filter.h"

#define control_dt (1 / 1000.0)

// in feet
#define ROBOT_DIAMETER (7.0/12.0)

/*
 * I am tired so I will re-read this later
 * http://blog.tkjelectronics.dk/2012/09/a-practical-approach-to-kalman-filter-and-how-to-implement-it/
 */

extern uint32_t state;
extern volatile uint16_t log_data;
extern uint16_t data_pos;
extern float recorded_data[4096];

sliding_window_decl(angle_history, 12);
sliding_window_decl(vel_history, 12);

struct RobotCommand {
    float cmd_vel;
    float cmd_omega;
    uint16_t age;
};

typedef struct RobotCommand RobotCommand;

typedef struct {
    eQEP* left_encoder;
    eQEP* right_encoder;
    angle_history left_encoder_filter;
    angle_history right_encoder_filter;
    angle_history left_angle_history;
    angle_history right_angle_history;
    vel_history left_velocity_filter;
    vel_history right_velocity_filter;
    vel_history left_velocity_history;
    vel_history right_velocity_history;
    vel_history angle_history;
    //vel_history angular_velocity_filter;
    MPU9250* imu;
    DAN* dan;
    RobotCommand* cmd;  // lol volatile

    // kalman filter
    float angle_filtered;
    float v_filtered;
    float angle_raw;
    int16_t rotations;
    float gyro_accum;   // Testing for comparison only, bad signal
    float rate;
    float rate_prev;
    float bias_est;
    float p00;
    float p01;
    float p10;
    float p11;
    float innovation;

    // gains
    float Kp_drive;
    float Ki_drive;

    // other measurements
    float t_left;
    float t_right;
    float w_left;
    float w_right;
    float a_left;
    float a_right;

    // setpoints
    float turn_target;
    float control_target;

    // controller states.
    float left_I;
    float right_I;

    // control outputs. READ ONLY
    float left_u;
    float right_u;

    // Odometry. READ ONLY
    float x_pos;
    float y_pos;
    float heading;
    float v_forward;
    float angular_velocity;
} Robot;

extern Robot robot;

void Robot_init(Robot* this, eQEP* left_encoder, eQEP* right_encoder, MPU9250* imu, DAN* dan);

void Robot_update(Robot* this);
