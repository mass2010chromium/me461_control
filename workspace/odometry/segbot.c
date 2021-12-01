/*
 * segbot.c
 *
 *  Created on: Nov 20, 2021
 *      Author: jcpen
 */

#include <robot.h>
uint32_t state = 0;
volatile uint16_t log_data = 0;
uint16_t data_pos = 0;
float recorded_data[4096];

Robot robot;

#define filt_old (-(125*control_dt - 2) / (125*control_dt + 2))
#define filt_new (250 / (125*control_dt + 2))
#define Q_ANGLE 0.001
#define Q_BIAS 0.003
#define R_MEASURE 0.3
#define CONTROL_TARGET 1.28
// #define CONTROL_TARGET 1.365

float b[ 3 ] = { 0.06745527388907189558775456816875,
    0.13491054777814379117550913633750,
    0.06745527388907189558775456816875 };
float a_rev[ 3 ] = { 1.0,
     0.41280159809618877098102984746220,
    -1.14298050253990113311886034352938 };

float derivative [ 3 ] = {0.5, -2, 1.5};
float derivative2 [ 4 ] = {-1, 4, -5, 2};

void Robot_init(Robot* this, eQEP* left_encoder, eQEP* right_encoder, MPU9250* imu, DAN* dan) {
    memset(this, 0, sizeof(Robot));
    this->left_encoder = left_encoder;
    this->right_encoder = right_encoder;
    this->imu = imu;
    this->dan = dan;
    this->angle_filtered = CONTROL_TARGET;
    this->angle_raw = CONTROL_TARGET;
    this->control_target = CONTROL_TARGET;
    this->Kp_turn = 0.4;
    this->Kv_turn = 0.1;
    this->Kp_drive = 0.001;
    this->Kv_drive = 0.00001;
    this->Kp_bal = 6;
    this->Kv_bal = -0.12;
    this->Kt_bal = -0.36;
    sliding_window_setup(this->left_encoder_filter);
    this->left_encoder_filter.w.pos = 0;
    sliding_window_setup(this->right_encoder_filter);
    this->right_encoder_filter.w.pos = 1;
    sliding_window_setup(this->left_angle_history);
    this->left_angle_history.w.pos = 2;
    sliding_window_setup(this->right_angle_history);
    this->right_angle_history.w.pos = 3;
    sliding_window_setup(this->turn_control_history);
    this->turn_control_history.w.pos = 4;
}

void Robot_update(Robot* this) {
    float new_left = read_eQEP(this->left_encoder);
    sliding_window_push(&this->left_encoder_filter, new_left);
    this->t_left = filter2(b, a_rev, arraysize(b), &this->left_encoder_filter,
                           &this->left_angle_history, this->left_angle_history.w.size);
    this->w_left = filter(derivative, arraysize(derivative),
                          &this->left_angle_history, this->left_angle_history.w.size) * (1/control_dt);
    this->a_left = filter(derivative2, arraysize(derivative2),
                          &this->left_angle_history, this->left_angle_history.w.size) * (1/control_dt) * (1/control_dt);
    float new_right = read_eQEP(this->right_encoder);
    sliding_window_push(&this->right_encoder_filter, new_right);
    this->t_right = filter2(b, a_rev, arraysize(b), &this->right_encoder_filter,
                            &this->right_angle_history, this->right_angle_history.w.size);
    this->w_right = filter(derivative, arraysize(derivative),
                           &this->right_angle_history, this->right_angle_history.w.size) * (1/control_dt);
    this->a_right = filter(derivative2, arraysize(derivative2),
                           &this->right_angle_history, this->right_angle_history.w.size) * (1/control_dt) * (1/control_dt);

    float turn_control = this->t_left - this->t_right;
    sliding_window_push(&this->turn_control_history, turn_control);
    float v_turn = filter(derivative, arraysize(derivative),
                          &this->turn_control_history, this->turn_control_history.w.size) * (1/control_dt);

    float turn_error = this->turn_target - turn_control;
    float turn_drive = this->Kp_turn * turn_error - this->Kv_turn * v_turn;
    if (turn_drive > 0.4) { turn_drive = 0.4; }
    else if (turn_drive < -0.4) { turn_drive = -0.4; }

    if (this->imu->status == MPU9250_IDLE) {
        MPU9250_update_stats(this->imu);

        // lol kalman filter
        this->rate = this->imu->gyro_x - this->bias_est;
        this->angle_filtered += this->rate * control_dt;
        this->p00 += control_dt * (this->p11 - this->p01 - this->p10 + Q_ANGLE);
        this->p01 -= control_dt * this->p11;
        this->p10 -= control_dt * this->p11;
        this->p11 += control_dt * Q_BIAS;

        this->angle_raw = __atan2(this->imu->accel_y, this->imu->accel_z);
        float innovation = this->angle_raw - this->angle_filtered;
        // TODO rollover hack, track how many times this happens
        if (innovation > 3*M_PI/2) {
            innovation -= 2*M_PI;
        }
        else if (innovation < -3*M_PI/2) {
            innovation += 2*M_PI;
        }
        float S = this->p00 + R_MEASURE;
        float k0 = this->p00 / S;
        float k1 = this->p10 / S;

        this->angle_filtered += k0 * innovation;
        this->bias_est += k1 * innovation;

        this->p11 -= k1 * this->p01;
        this->p10 -= k1 * this->p00;
        this->p01 -= k0 * this->p01;
        this->p00 -= k0 * this->p00;

        this->rate_prev = this->rate;

        float e_left = this->angle_filtered - this->control_target;
        float e_right = this->angle_filtered - this->control_target;
        float left_command = this->Kp_bal*e_left - this->Kv_bal * this->w_left - this->Kt_bal * this->rate;
        float right_command = this->Kp_bal*e_right - this->Kv_bal * this->w_right - this->Kt_bal * this->rate;


        left_command = left_command / 2 + turn_drive;
        right_command = right_command / 2 -turn_drive;

        //http://software-dl.ti.com/C2000/docs/optimization_guide/phase1/saturation.html
        left_command = (left_command > 1) ? 1 : left_command;
        left_command = (left_command < -1) ? -1 : left_command;
        right_command = (right_command > 1) ? 1 : right_command;
        right_command = (right_command < -1) ? -1 : right_command;

        this->left_u = left_command;
        this->right_u = right_command;

        // TESTING CODE
        ++state;
        float drive = 0;
        log_data = 1;
        if (state > 3000 && state < 3500) {
            drive = 0.5;
            this->left_u = -drive;
        }
        // DAN_set_write(this->dan, -drive, drive);
        // END TESTING CODE

        DAN_set_write(this->dan, -left_command, right_command);
        MPU9250_set_read_accel_gyro(this->imu);
    }
}
