/*
 * pwm.h
 *
 *  Created on: 14 July 2017
 *      Author: Steve Chang
 */

#ifndef _PWM_H_
#define _PWM_H_
#include "stm32f10x.h"

#define PwmPulseMax 5000 /* 200 Hz */
//#define PwmPulseMax 2500 /* 400 Hz */

void Pwm1_Pulse(uint16_t pulse);
void Pwm1_Reverse();

void Pwm_Init(void);
void Pwm_Pulse(uint8_t channel, uint16_t pulse);

#endif