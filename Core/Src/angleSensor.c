/*
 * angleSensor.c
 *
 *  Created on: 4 авг. 2022 г.
 *      Author: jkcvk
 */

#include "angleSensor.h"
#include "utils.h"
#include "string.h"
#include "stdio.h"

extern UART_HandleTypeDef huart1;

void angleSensor_init(angleSensor *as, TIM_HandleTypeDef *enc, TIM_HandleTypeDef *vel, int ppr) {
	as->enc_timer = enc;
	as->vel_timer = vel;
	as->old_us = 0;
	as->us = 0;
	as->diff_s = 0.0f;
	as->ppr = ppr;
	as->cpr = ppr * 4;
	as->enabled = 0;
	as->prev_counter = 0;
	as->one_pulse = _2PI * (1.0f)/((float) ppr);
}

void angleSensor_start(angleSensor *as) {
	HAL_TIM_Encoder_Start(as->enc_timer,TIM_CHANNEL_ALL);
	HAL_TIM_IC_Start_IT(as->vel_timer, TIM_CHANNEL_1);
	as->enabled = 1;
}

void angleSensor_stop(angleSensor *as) {
	HAL_TIM_Encoder_Stop(as->enc_timer,TIM_CHANNEL_ALL);
	HAL_TIM_IC_Stop(as->vel_timer, TIM_CHANNEL_1);
	as->enabled = 0;
}

void angleSensor_velocityMeasurementLoop(angleSensor *as) {
	if (as->enabled != 1) return;
	as->old_us = as->us;
	as->us = HAL_TIM_ReadCapturedValue(as->vel_timer, TIM_CHANNEL_1);
	as->vel_timer->Instance->CCR1 = 0;
	as->diff_s = (as->us - as->old_us) * 1e-6;
}

float angleSensor_getAngle(angleSensor *as) {
	if (as->enabled != 1) return 0.0f;
	int32_t pulseCounter = -unwrap_encoder((uint16_t) as->enc_timer->Instance->CNT, &as->prev_counter);
	return _2PI * (pulseCounter) / ((float)as->cpr);
}

uint32_t angleSensor_getRawAngle(angleSensor *as) {
	return unwrap_encoder((uint16_t) as->enc_timer->Instance->CNT, &as->prev_counter);
}

uint16_t angleSensor_getCounter(angleSensor *as) {
	if (as->enabled != 1) return 0;
	return (uint16_t) as->enc_timer->Instance->CNT;
}

float angleSensor_getVelocity(angleSensor *as) {
	if (as->enabled != 1) return 0.0f;
	//if (as->diff_s < 0.0001) return 0.0f;
	return (as->one_pulse) / (as->diff_s);
	return as->diff_s;
}


int32_t unwrap_encoder(uint16_t in, int32_t * prev)
{
    int32_t c32 = (int32_t)in - HALF_PERIOD;    //remove half period to determine (+/-) sign of the wrap
    int32_t dif = (c32-*prev);  //core concept: prev + (current - prev) = current

    //wrap difference from -HALF_PERIOD to HALF_PERIOD. modulo prevents differences after the wrap from having an incorrect result
    int32_t mod_dif = ((dif + HALF_PERIOD) % ONE_PERIOD) - HALF_PERIOD;
    if(dif < -HALF_PERIOD)
        mod_dif += ONE_PERIOD;  //account for mod of negative number behavior in C

    int32_t unwrapped = *prev + mod_dif;
    *prev = unwrapped;  //load previous value

    return unwrapped + HALF_PERIOD; //remove the shift we applied at the beginning, and return
}

void angleSensor_debugLog(angleSensor *as) {
	char trans_str[64];
	snprintf(trans_str, 63, "%d;\n", (int) as->enc_timer->Instance->CNT);
	HAL_UART_Transmit(&huart1, (uint8_t*)trans_str, strlen(trans_str), 1000);
}
