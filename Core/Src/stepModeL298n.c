/*
 * stepMode.c
 *
 *  Created on: 16 июн. 2022 г.
 *      Author: jkcvk
 */


#include <stepModeL298n.h>

uint16_t sequencerHalfStep[6][8] = {
			{1, 1, 0, 0, 0, 0, 0, 1},
			{0, 0, 0, 1, 1, 1, 0, 0},
			{0, 1, 1, 1, 0, 0, 0, 0},
			{0, 0, 0, 0, 0, 1, 1, 1},
			{95, 70, 10, 70, 95, 70, 10, 70},
			{10, 70, 95, 70, 10, 70, 95, 70}
	};

uint16_t sequncerFullStep[6][4] = {
		{1, 0, 0, 0},
		{0, 0, 1, 0},
		{0, 1, 0, 0},
		{0, 0, 0, 1},
		{95, 95, 95, 95},
		{95, 95, 95, 95}
};


void initStepMode(stepMode *s, TIM_HandleTypeDef *pwm_htim,
		GPIO_TypeDef* GPIOx, uint16_t IN1, uint16_t IN2, uint16_t IN3, uint16_t IN4){
	s->pwm_timer = pwm_htim;
	s->port = GPIOx;
	s->in1 = IN1;
	s->in2 = IN2;
	s->in3 = IN3;
	s->in4 = IN4;
	s->steps = 0;
	s->ticks = 0;
	s->freq_timer = 0;
}

void setFrequencyTim(stepMode *s, TIM_HandleTypeDef *freq_htim) {
	s->freq_timer = freq_htim;
}

void updateFreqTimArr (stepMode *s, uint16_t arr) {
	if (s->freq_timer == 0 ) return;
    __HAL_TIM_SET_AUTORELOAD(s->freq_timer, arr);
    if (__HAL_TIM_GET_COUNTER(s->freq_timer) >= __HAL_TIM_GET_AUTORELOAD(s->freq_timer)) {
    	s->freq_timer->Instance->EGR  |= TIM_EGR_UG;
    }
}

void setSteps(stepMode *s, uint32_t steps) {
	s->ticks = 0;
	s->steps = steps;
}

void stopStepMode(stepMode *s) {
	s->steps = 0;
}

void halfStepModeLoop(stepMode *s) {
	if (s->steps <= s->ticks) {
		HAL_GPIO_WritePin(s->port, s->in1, 0);
		HAL_GPIO_WritePin(s->port, s->in2, 0);
		HAL_GPIO_WritePin(s->port, s->in3, 0);
		HAL_GPIO_WritePin(s->port, s->in4, 0);
		s->pwm_timer->Instance->CCR1 = 0;
		s->pwm_timer->Instance->CCR2 = 0;
		return;
	}
	HAL_GPIO_WritePin(s->port, s->in1, sequencerHalfStep[0][s->ticks % 8]);
	HAL_GPIO_WritePin(s->port, s->in2, sequencerHalfStep[1][s->ticks % 8]);
	s->pwm_timer->Instance->CCR1 = sequencerHalfStep[4][s->ticks % 8];
	HAL_GPIO_WritePin(s->port, s->in3, sequencerHalfStep[2][s->ticks % 8]);
	HAL_GPIO_WritePin(s->port, s->in4, sequencerHalfStep[3][s->ticks % 8]);
	s->pwm_timer->Instance->CCR2 = sequencerHalfStep[5][s->ticks % 8];

	s->ticks++;
}

void fullStepModeLoop(stepMode *s) {
	if (s->steps <= s->ticks) {
		HAL_GPIO_WritePin(s->port, s->in1, 0);
		HAL_GPIO_WritePin(s->port, s->in2, 0);
		HAL_GPIO_WritePin(s->port, s->in3, 0);
		HAL_GPIO_WritePin(s->port, s->in4, 0);
		s->pwm_timer->Instance->CCR1 = 0;
		s->pwm_timer->Instance->CCR2 = 0;
		return;
	}
	HAL_GPIO_WritePin(s->port, s->in1, sequncerFullStep[0][s->ticks % 4]);
	HAL_GPIO_WritePin(s->port, s->in2, sequncerFullStep[1][s->ticks % 4]);
	s->pwm_timer->Instance->CCR1 = sequncerFullStep[4][s->ticks % 4];
	HAL_GPIO_WritePin(s->port, s->in3, sequncerFullStep[2][s->ticks % 4]);
	HAL_GPIO_WritePin(s->port, s->in4, sequncerFullStep[3][s->ticks % 4]);
	s->pwm_timer->Instance->CCR2 = sequncerFullStep[5][s->ticks % 4];
	s->ticks++;

}
