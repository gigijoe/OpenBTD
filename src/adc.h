/*
 * adc.h
 *
 *  Created on: 31 Oct 2016
 *      Author: Steve Chang
 */

#ifndef __ADC_H
#define	__ADC_H


#include "stm32f10x.h"

#define NUM_CHANNEL 2
extern __IO uint16_t ADC_SLOT[NUM_CHANNEL];

void ADC1_Init(void);

#endif /* __ADC_H */
