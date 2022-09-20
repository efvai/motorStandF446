/*
 * currentSensor.h
 *
 *  Created on: Jun 29, 2022
 *      Author: jkcvk
 */

#ifndef INC_CURRENTSENSOR_H_
#define INC_CURRENTSENSOR_H_

#include "main.h"

#define CALIBRATE_MEASUREMENT_COUNT 100

typedef struct {
	ADC_HandleTypeDef *adc;
	DMA_HandleTypeDef *dma_adc;

	uint16_t raw_value[1];
	float offset;
	float voltage_offset;

	uint8_t enabled;
} currentSensor;

void currentSensor_init(currentSensor *cs, ADC_HandleTypeDef *hadc,
		DMA_HandleTypeDef *hdma_adc);
void currentSensor_calibrate(currentSensor *cs);
void currentSensor_start(currentSensor *cs);
void currentSensor_stop(currentSensor *cs);
float currentSensor_getCurrent(currentSensor *cs);
uint16_t currentSensor_getRawAdcData(currentSensor *cs);

// For internal usage
float current_convertion(uint16_t raw_value, float volt_offset);
float voltage_convertion(uint16_t raw_value);
void currentSensor_log(currentSensor *cs);

#endif /* INC_CURRENTSENSOR_H_ */
