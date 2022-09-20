/*
 * hbridgeDriver.h
 *
 *  Created on: 27 июл. 2022 г.
 *      Author: jkcvk
 */

#ifndef INC_STEPPERDRIVER2PWM_H_
#define INC_STEPPERDRIVER2PWM_H_

#include "main.h"

#define CCR_VALUE 1000

#define ABS(x) ((x) > 0 ? (x) : -(x))
#define CONSTRAIN(amt, low, high) ((amt)<(low)?(low):((amt)>(high)?(high):(amt)))

typedef struct {
	TIM_HandleTypeDef *pwm_timer;
	GPIO_TypeDef* port;
	uint16_t dir1;
	uint16_t dir2;

	uint16_t supply_voltage;
	uint16_t limit_voltage;

	uint8_t is_init;

} stepperDriver2Pwm;

void stepperDriver2Pwm_setSupplyVoltage(stepperDriver2Pwm *sd2m, uint16_t v);
void stepperDriver2Pwm_setLimitVoltage(stepperDriver2Pwm *sd2m, uint16_t v);
void stepperDriver2Pwm_init(stepperDriver2Pwm *sd2m, TIM_HandleTypeDef *PWM_HTIM,
		GPIO_TypeDef* GPIOx, uint16_t DIR1, uint16_t DIR2);
void stepperDriver2Pwm_disable(stepperDriver2Pwm *sd2m);
void stepperDriver2Pwm_setPwm(stepperDriver2Pwm *sd2m, float Ua, float Ub);


#endif /* INC_STEPPERDRIVER2PWM_H_ */
