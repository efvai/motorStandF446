/*
 * stepperDriver2Pwm.c
 *
 *  Created on: 27 июл. 2022 г.
 *      Author: jkcvk
 */

#include "stepperDriver2Pwm.h"

//{1, 0, -1,  0},
//{0, 1,  0, -1},

void stepperDriver2Pwm_init(stepperDriver2Pwm *sd2m, TIM_HandleTypeDef *PWM_HTIM,
		GPIO_TypeDef* GPIOx, uint16_t DIR1, uint16_t DIR2) {
	sd2m->pwm_timer = PWM_HTIM;
	sd2m->port = GPIOx;
	sd2m->dir1 = DIR1;
	sd2m->dir2 = DIR2;
	HAL_TIM_PWM_Start(sd2m->pwm_timer, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(sd2m->pwm_timer, TIM_CHANNEL_2);
	sd2m->pwm_timer->Instance->CCR1 = CCR_VALUE;
	sd2m->pwm_timer->Instance->CCR2 = CCR_VALUE;
	HAL_GPIO_WritePin(sd2m->port, sd2m->dir1, 0);
	HAL_GPIO_WritePin(sd2m->port, sd2m->dir2, 0);
	sd2m->supply_voltage = 12;
	sd2m->limit_voltage = 7;
	sd2m->is_init = 1;
}

void stepperDriver2Pwm_setSupplyVoltage(stepperDriver2Pwm *sd2m, uint16_t v) {
	sd2m->supply_voltage = v;
}

void stepperDriver2Pwm_setLimitVoltage(stepperDriver2Pwm *sd2m, uint16_t v) {
	sd2m->limit_voltage = v;
}

void stepperDriver2Pwm_disable(stepperDriver2Pwm *sd2m) {
	stepperDriver2Pwm_setPwm(sd2m, 0, 0);
	HAL_GPIO_WritePin(sd2m->port, sd2m->dir1, 0);
	HAL_GPIO_WritePin(sd2m->port, sd2m->dir2, 0);
}

void stepperDriver2Pwm_setPwm(stepperDriver2Pwm *sd2m, float Ua, float Ub) {
	float duty1 = 0.0f;
	float duty2 = 0.0f;
	Ua = CONSTRAIN(Ua, -sd2m->limit_voltage, sd2m->limit_voltage);
	Ub = CONSTRAIN(Ub, -sd2m->limit_voltage, sd2m->limit_voltage);

	duty1 = CONSTRAIN(ABS(Ua)/sd2m->supply_voltage, 0.0f, 1.0f);
	duty2 = CONSTRAIN(ABS(Ub)/sd2m->supply_voltage, 0.0f, 1.0f);

	HAL_GPIO_WritePin(sd2m->port, sd2m->dir1, Ua > 0 ? 1 : 0);
	HAL_GPIO_WritePin(sd2m->port, sd2m->dir2, Ub > 0 ? 1 : 0);

	sd2m->pwm_timer->Instance->CCR1 = CCR_VALUE - (duty1 * CCR_VALUE);
	sd2m->pwm_timer->Instance->CCR2 = CCR_VALUE - (duty2 * CCR_VALUE);
}
