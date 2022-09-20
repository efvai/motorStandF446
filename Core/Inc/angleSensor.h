/*
 * angleSensor.h
 *
 *  Created on: 4 авг. 2022 г.
 *      Author: jkcvk
 *
 *      Encoder E40H12-5000-6-L-5
 *      @params
 *      1) 5000 pulse/revolution
 *      2) Line driver output
 *      3) VCC = 5
 *      4) A->B - Clockwise(CW); B->A CounterClockWise(CCW)
 *      @pins
 *      Black - A ; Red - ~A - PC6
 *   	White - B ; Gray - ~B - PC7
 *   	Orange - Z ; Yellow - ~Z - PD15
 *   	Brown - +V (5V)
 *   	Blue - GND (0V)
 */

#ifndef INC_ANGLESENSOR_H_
#define INC_ANGLESENSOR_H_


#define ONE_PERIOD 65536
#define HALF_PERIOD 32768

#include "main.h"


typedef struct {
	TIM_HandleTypeDef *enc_timer;
	TIM_HandleTypeDef *vel_timer;
	GPIO_TypeDef* index_port;
	uint16_t index_pin;

	int ppr;
	int cpr;

	float diff_s;
	float one_pulse;
	uint32_t old_us;
	uint32_t us;
	uint8_t enabled;

	int32_t prev_counter;
} angleSensor;

void angleSensor_init(angleSensor *as, TIM_HandleTypeDef *enc, TIM_HandleTypeDef *vel, int ppr);
void angleSensor_start(angleSensor *as);
void angleSensor_stop(angleSensor *as);

void angleSensor_velocityMeasurementLoop(angleSensor *as);
float angleSensor_getAngle(angleSensor *as);
float angleSensor_getVelocity(angleSensor *as);

uint32_t angleSensor_getRawAngle(angleSensor *as);
uint16_t angleSensor_getCounter(angleSensor *as);


int32_t unwrap_encoder(uint16_t in, int32_t * prev);
void angleSensor_debugLog(angleSensor *as);

#endif /* INC_ANGLESENSOR_H_ */
