/*
 * pwm.h
 *
 *  Created on: Oct 1, 2021
 *      Author: jcpen
 */
#ifndef PWM_H_IGNORE_
#define PWM_H_IGNORE_

#include <stdio.h>
#include <stdlib.h>
#include <stdarg.h>
#include <string.h>
#include <math.h>
#include <limits.h>
#include "F28x_Project.h"
#include "F2837xD_epwm.h"

typedef struct {
    volatile union TBCTL_REG* tbctl;
    volatile uint16_t* prd;
    volatile union CMPA_REG* cmpa;
    volatile union CMPB_REG* cmpb;
    volatile union AQCTLA_REG* aqctla;
    volatile union AQCTLB_REG* aqctlb;
    volatile union ETSEL_REG* etsel;
    volatile union ETPS_REG* etps;
    float control_min;  // Input scale min
    float control_max;  // Input scale max
    float control_scale_inv;
    float output_min;   // output duty cycle min
    float output_max;   // output duty cycle max
} PWM;

#define setupPWM(pwm_id, pwm_struct, period) \
EPwm ## pwm_id ## Regs.TBCTL.bit.FREE_SOFT = 0b10; /* Free run mode: 1x */\
EPwm ## pwm_id ## Regs.TBCTL.bit.CLKDIV = 0b000;   /* Divide by 1 (default 50Mhz) */\
EPwm ## pwm_id ## Regs.TBCTL.bit.PHSEN = 0b0;      /* Disable phase loading */\
EPwm ## pwm_id ## Regs.TBCTL.bit.CTRMODE = 0b11;   /* Disable counter */\
EPwm ## pwm_id ## Regs.TBCTR = 0;                  /* Counter start at 0 */\
EPwm ## pwm_id ## Regs.TBPRD = (period);           /* Set period */\
EPwm ## pwm_id ## Regs.TBPHS.bit.TBPHS = 0;        /* Phase = 0 */\
EPwm ## pwm_id ## Regs.CMPA.bit.CMPA = 0;          /* Trigger when TBCTR == CMPA. Update this! */\
EPwm ## pwm_id ## Regs.CMPB.bit.CMPB = 0;          /* Trigger when TBCTR == CMPA. Update this! */\
EPwm ## pwm_id ## Regs.AQCTLA.bit.CAU = 0b01;      /* Clear output on CMPA match */\
EPwm ## pwm_id ## Regs.AQCTLA.bit.ZRO = 0b10;      /* Set output when TBCTR == 0 */\
EPwm ## pwm_id ## Regs.AQCTLB.bit.CBU = 0b01;      /* Clear output on CMPB match */\
EPwm ## pwm_id ## Regs.AQCTLB.bit.ZRO = 0b10;      /* Set output when TBCTR == 0 */\
EPwm ## pwm_id ## Regs.TBCTL.bit.CTRMODE = 0b00;   /* Count-up mode */\
(pwm_struct)->tbctl = & EPwm ## pwm_id ## Regs.TBCTL;\
(pwm_struct)->prd = & EPwm ## pwm_id ## Regs.TBPRD;\
(pwm_struct)->cmpa = & EPwm ## pwm_id ## Regs.CMPA;\
(pwm_struct)->cmpb = & EPwm ## pwm_id ## Regs.CMPB;\
(pwm_struct)->aqctla = & EPwm ## pwm_id ## Regs.AQCTLA;\
(pwm_struct)->aqctlb = & EPwm ## pwm_id ## Regs.AQCTLB;\
(pwm_struct)->etsel = & EPwm ## pwm_id ## Regs.ETSEL;\
(pwm_struct)->etps = & EPwm ## pwm_id ## Regs.ETPS;\
setPWMControlScale((pwm_struct), -10, 10, 0, 1);

/**
 * Setup the input and output control scaling for a PWM output.
 */
void setPWMControlScale(PWM* pwm, float min, float max,
                        float out_min, float out_max);

void setPWMA(PWM* pwm, float control_effort);

void setPWMB(PWM* pwm, float control_effort);

#endif /* PWM_H_IGNORE_ */
