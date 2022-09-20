/*
 * modbusCommander.h
 *
 *  Created on: Aug 17, 2022
 *      Author: jkcvk
 */

#ifndef INC_MODBUSCOMMANDER_H_
#define INC_MODBUSCOMMANDER_H_

#include "utils.h"

void modbusCommander_bindParameter(param_float* param, uint16_t reg_index);
void modbusCommander_bindParameter(param_int* param, uint16_t reg_index);

void modbusCommander_writeMotorTarget(float target);
void modbusCommander_writeCurrentSensorAOffset(float offset);
void modbusCommander_writeCurrentSensorBOffset(float offset);
void modbusCommander_writeControlMode(uint16_t mode);
void modbusCommander_writeVoltageLimit(float limit);
void modbusCommander_writeVoltageSupply(float supply);
void modbusCommander_writeCurrentLimit(float limit);
void modbusCommander_writeVelocityLimit(float limit);
void modbusCommander_writeMotorEnabled(uint16_t en);
void modbusCommander_writePiGains(float c_p, float c_i, float v_p, float v_i, float a_p, float a_i);
void modbusCommander_writeZeroAngleOffset(float off);


#endif /* INC_MODBUSCOMMANDER_H_ */
