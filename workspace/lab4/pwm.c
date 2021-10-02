/*
 * pwm.c
 *
 *  Created on: Oct 1, 2021
 *      Author: jcpen
 */
#include <pwm.h>

/**
 * Setup the input and output control scaling for a PWM output.
 */
void setPWMControlScale(PWM* pwm, float min, float max,
                        float out_min, float out_max) {
    pwm->control_min = min;
    pwm->control_max = max;
    pwm->control_scale_inv = 1.0f / (max - min);
    pwm->output_min = out_min;
    pwm->output_max = out_max;
}

/**
 * Set the A output channel to output with the specified control effort.
 * Scales accoring to scaling set by `setPWMControlScale`.
 */
void setPWMA(PWM* pwm, float control_effort) {
    float control_scale = pwm->control_max - pwm->control_min;
    float output_scale = pwm->output_max - pwm->output_min;
    control_effort -= pwm->control_min;
    if (control_effort < 0) control_effort = 0;
    if (control_effort > control_scale) control_effort = control_scale;
    control_effort = (control_effort * pwm->control_scale_inv * output_scale
            + pwm->output_min) * (*pwm->prd);
    pwm->cmpa->bit.CMPA = control_effort;
}

/**
 * Set the B output channel to output with the specified control effort.
 * Scales accoring to scaling set by `setPWMControlScale`.
 */
void setPWMB(PWM* pwm, float control_effort) {
    float control_scale = pwm->control_max - pwm->control_min;
    float output_scale = pwm->output_max - pwm->output_min;
    control_effort -= pwm->control_min;
    if (control_effort < 0) control_effort = 0;
    if (control_effort > control_scale) control_effort = control_scale;
    control_effort = (control_effort * pwm->control_scale_inv * output_scale
            + pwm->output_min) * (*pwm->prd);
    pwm->cmpb->bit.CMPB = control_effort;
}

