/*
 * modbusCommander.c
 *
 *  Created on: Aug 17, 2022
 *      Author: jkcvk
 */


#include "main.h"
#include "modbusCommander.h"

/* (0-1) = Motor Target
 *
 * (2-3) = Sensor A offset
 * (4-5) = Sensor B offset
 * (6-7) = Voltage limit
 * (8-9) = Current limit
 * (10-11) = Velocity limit
 * (12-13) = Voltage Supply
 * (14-15) = Control Mode
 * (16-17) = on off
 * (18-19) = current p gain
 * (20-21) = current i gain
 * (22-23) = velocity p gain
 * (24-25) = velocity i gain
 * (26-27) = angle p gain
 * (28-29) = angle i gain
 * (30-31) - zero angle offset
 */

void modbusCommander_bindParameter(param_float* param, uint16_t reg_index) {
	if (reg_index < HOLDING_REG_LENGTH)
		param->raw = usSRegHoldBuf[reg_index];
}

void modbusCommander_bindParameter(param_int* param, uint16_t reg_index) {
	if (reg_index < HOLDING_REG_LENGTH)
		param->raw = usSRegHoldBuf[reg_index];
}

void modbusCommander_writeZeroAngleOffset(float off) {
	converter c;
	c.flt = off;
	usSRegHoldBuf[30] = c.lo;
	usSRegHoldBuf[31] = c.hi;
}

void modbusCommander_writePiGains(float c_p, float c_i, float v_p, float v_i, float a_p, float a_i) {
	converter c;
	c.flt = c_p;
	usSRegHoldBuf[16] = c.lo;
	usSRegHoldBuf[17] = c.hi;
	c.flt = c_i;
	usSRegHoldBuf[18] = c.lo;
	usSRegHoldBuf[19] = c.hi;
	c.flt = v_p;
	usSRegHoldBuf[20] = c.lo;
	usSRegHoldBuf[21] = c.hi;
	c.flt = v_i;
	usSRegHoldBuf[22] = c.lo;
	usSRegHoldBuf[23] = c.hi;
	c.flt = a_p;
	usSRegHoldBuf[24] = c.lo;
	usSRegHoldBuf[25] = c.hi;
	c.flt = a_i;
	usSRegHoldBuf[26] = c.lo;
	usSRegHoldBuf[27] = c.hi;
}

void modbusCommander_writeMotorEnabled(uint16_t en) {
	usSRegHoldBuf[15] = en;
}

void modbusCommander_writeMotorTarget(float target) {
	converter c;
	c.flt = target;
	usSRegHoldBuf[0] = c.lo;
	usSRegHoldBuf[1] = c.hi;
}

void modbusCommander_writeCurrentSensorAOffset(float offset) {
	converter c;
	c.flt = offset;
	usSRegHoldBuf[2] = c.lo;
	usSRegHoldBuf[3] = c.hi;
}

void modbusCommander_writeCurrentSensorBOffset(float offset) {
	converter c;
	c.flt = offset;
	usSRegHoldBuf[4] = c.lo;
	usSRegHoldBuf[5] = c.hi;
}

void modbusCommander_writeControlMode(uint16_t mode) {
	usSRegHoldBuf[14] = mode;
}

void modbusCommander_writeVoltageLimit(float limit) {
	converter c;
	c.flt = limit;
	usSRegHoldBuf[6] = c.lo;
	usSRegHoldBuf[7] = c.hi;
}


void modbusCommander_writeCurrentLimit(float limit) {
	converter c;
	c.flt = limit;
	usSRegHoldBuf[8] = c.lo;
	usSRegHoldBuf[9] = c.hi;
}

void modbusCommander_writeVelocityLimit(float limit) {
	converter c;
	c.flt = limit;
	usSRegHoldBuf[10] = c.lo;
	usSRegHoldBuf[11] = c.hi;
}

void modbusCommander_writeVoltageSupply(float supply) {
	converter c;
	c.flt = supply;
	usSRegHoldBuf[12] = c.lo;
	usSRegHoldBuf[13] = c.hi;
}
