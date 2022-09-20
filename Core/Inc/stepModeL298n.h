/*
 * stepMode.h
 *
 *  Created on: 16 июн. 2022 г.
 *      Author: jkcvk
 */

#ifndef INC_STEPMODEL298N_H_
#define INC_STEPMODEL298N_H_

#include "main.h"



typedef struct {
	TIM_HandleTypeDef *pwm_timer;
	TIM_HandleTypeDef *freq_timer;
	GPIO_TypeDef* port;
	uint16_t in1;
	uint16_t in2;
	uint16_t in3;
	uint16_t in4;
	uint32_t steps;
	uint32_t ticks;
} stepMode;

void initStepMode(stepMode *s, TIM_HandleTypeDef *pwm_htim,
		GPIO_TypeDef* GPIOx, uint16_t IN1, uint16_t IN2, uint16_t IN3, uint16_t IN4); // 1 timer -> 2 channels (EN1, EN2)

void setSteps(stepMode *s, uint32_t steps);
void stopStepMode(stepMode *s);
void setFrequencyTim(stepMode *s, TIM_HandleTypeDef *freq_htim);
void updateFreqTimArr (stepMode *s, uint16_t arr);


void halfStepModeLoop(stepMode *s);
void fullStepModeLoop(stepMode *s);

#endif /* INC_STEPMODEL298N_H_ */
