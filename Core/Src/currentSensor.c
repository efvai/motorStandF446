/*
 * currentSensor.c
 *
 *  Created on: Jun 29, 2022
 *      Author: jkcvk
 */
/*
 * ACS712 5V
 *
 * @PARAMETERS:
 * vcc: 4.5 - 5.5 v
 * icc: 13 - 15 ma
 * zero current output voltage: vcc x 0.5 (2.5v)
 * @SENSITIVITY:
   +- 5 a: 185 mv/a (+-13513 mamper @ 5v)
   @theoretical resolution for 5v analog pin:
   (4.8828 mv)
   26.4ma
   @TOTAL ERROR @ 25c: +-1.5 %

   TODO: check adc trigger of falling edge
   TODO: попробовать у TIM1 выключить trigger source
 */

#include "currentSensor.h"
#include "string.h"
#include "stdio.h"

#define ADC_VCC 3.3
#define ADC_ACCURACY 4095.0
#define SENSIVITY 0.185
#define ZERO_CURRENT_VOLTAGE 2.5


uint16_t adc[1];
float offset;
extern UART_HandleTypeDef huart1;

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc) {

}


float current_convertion(uint16_t raw_value, float volt_offset) {
	float voltage = (raw_value / ADC_ACCURACY) * ADC_VCC;
	float off = ZERO_CURRENT_VOLTAGE - offset;
	return (voltage - off) / SENSIVITY;
}

float voltage_convertion(uint16_t raw_value) {
	return (raw_value / ADC_ACCURACY) * ADC_VCC;
}

void currentSensor_init(currentSensor *cs, ADC_HandleTypeDef *hadc,
		DMA_HandleTypeDef *hdma_adc) {
	cs->adc = hadc;
	cs->dma_adc = hdma_adc;
	cs->offset = 0;
	cs->voltage_offset = 0;
	cs->enabled = 0;
	cs->raw_value[0] = 0;
}

//default offset = -0.66 A
void currentSensor_calibrate(currentSensor *cs) {
	cs->adc->Instance->CR2 &= ~ADC_CR2_EXTEN;
	int average = 0;
	uint16_t rawValues[CALIBRATE_MEASUREMENT_COUNT];
	for (uint16_t i = 0; i < CALIBRATE_MEASUREMENT_COUNT; i++) {
		HAL_ADC_Start(cs->adc);
		HAL_ADC_PollForConversion(cs->adc, 200);
		rawValues[i] = HAL_ADC_GetValue(cs->adc);
		HAL_ADC_Stop(cs->adc);
		average += rawValues[i];
	}
	cs->adc->Instance->CR2 |= ADC_CR2_EXTEN_1 | ADC_CR2_EXTEN_0;
//	char trans_str[64];
	average /= CALIBRATE_MEASUREMENT_COUNT;
//	snprintf(trans_str, 63, "Zero current Voltage %f\n", voltage_convertion(average));
//	HAL_UART_Transmit(&huart2, (uint8_t*)trans_str, strlen(trans_str), 1000);
	cs->voltage_offset = ZERO_CURRENT_VOLTAGE - voltage_convertion(average);
	cs->offset = current_convertion(average, 0.0);
	offset = cs->voltage_offset;
//	snprintf(trans_str, 63, "Voltage offset %f\n", cs->voltage_offset);
//	HAL_UART_Transmit(&huart2, (uint8_t*)trans_str, strlen(trans_str), 1000);


}

void currentSensor_start(currentSensor *cs) {
	HAL_ADC_Start_DMA(cs->adc, (uint32_t *) cs->raw_value, 1);
	cs->enabled = 1;
}

void currentSensor_stop(currentSensor *cs) {
	HAL_ADC_Stop_DMA(cs->adc);
	cs->enabled = 0;
}

float currentSensor_getCurrent(currentSensor *cs) {
	if (cs->enabled != 1) return 0.0f;
	if (cs->raw_value[0] == 0) return 0.0f;
	return current_convertion(cs->raw_value[0], cs->voltage_offset);
}

uint16_t currentSensor_getRawAdcData(currentSensor *cs) {
	if (cs->enabled != 1) return 0;
	return cs->raw_value[0];
}

void currentSensor_log(currentSensor *cs) {
	//if (cs->enabled != 1) return;
		//char trans_str[64];
		//float current = current_convertion(cs->raw_value[0], cs->voltage_offset);
		//snprintf(trans_str, 63, "%f; %f\n", voltage_convertion(cs->raw_value[0]), current);
		//HAL_UART_Transmit(&huart1, (uint8_t*)trans_str, strlen(trans_str), 1000);
}
