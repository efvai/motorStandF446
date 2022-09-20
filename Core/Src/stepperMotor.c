/*
 * stepperMotor.c
 *
 *  Created on: 28 июл. 2022 г.
 *      Author: jkcvk
 */

#include "stepperMotor.h"
#include "modbusCommander.h"
#include <math.h>

int8_t sequencerFullStep[2][4] = {
		{1, 0, -1,  0},
		{0, 1,  0, -1},
};

/* Init PID regulator
	@params
		pi* - pointer to empty pi struct
		P - Proportional term
		I - Integral term
		limit - saturation of output */
void pi_init(pi_reg* p, float kp, float ki, float lim) {
	p->P = kp;
	p->I = ki;
	p->limit = lim;
	p->errorPrev = 0.0;
	p->integralPrev = 0.0;
	p->ts = TS_CUR * 1e-6;
}

void pi_setTs(pi_reg *p, float ts) {
	p->ts = ts;
}

void pi_setLimit(pi_reg *p, float lim) {
	p->limit = lim;
}

float pi_activate(pi_reg* p, float err) {
	// Discrete implementations
	// proportional part
	// u_p  = P *e(k)
	float proportional = p->P * err;
	// Integral part
	// u_ik = u_ik_1  + I*Ts/2*(ek + ek_1)
	double integral = p->integralPrev + p->I * p->ts * 0.5 * (err + p->errorPrev);
	// antiwindup - limit the output
	integral = CONSTRAIN(integral, -p->limit, p->limit);
	// sum all parts and limit sum
	float output = proportional + integral;
	output = CONSTRAIN(output, -p->limit, p->limit);
	//saving
	p->integralPrev = integral;
	p->errorPrev = err;
	return output;
}

void lowPassFilter_init(low_pass_filter *lpf, float tf, float ts) {
	lpf->time_constant = tf;
	lpf->ts = ts;
	lpf->y_prev = 0.0f;
}

float lowPassFilter_activate(low_pass_filter *lpf, float x) {
	float alpha = lpf->time_constant / (lpf->time_constant + lpf->ts);
	float y = alpha * lpf->y_prev + (1.0f - alpha) * x;
	lpf->y_prev = y;
	return y;
}

void stepperMotor_init(stepperMotor *m) {
	m->sd2p = NULL;
	m->cs_a = NULL;
	m->cs_b = NULL;
	m->as = NULL;

	m->polePairs = 50;
	m->voltageLimit = 12;
	m->velLimit = defvellim;
	m->currentLimit = 1;
	m->controlMode = TORQUE_CONTROL;
	m->dqVoltage.q = 0.0;
	m->dqVoltage.d = 0.0;
	m->dqCurrent.q = 0.0;
	m->dqCurrent.d = 0.0;
	m->abVoltage.a = 0.0;
	m->abVoltage.b = 0.0;
	m->abCurrent.a = 0.0;
	m->abCurrent.b = 0.0;
	m->currentDes = 0.0;
	m->target = 0.0;
	m->shaftAngle = 0.0;
	m->zeroElectricalAngle = 0.0;
	pi_init(&m->qReg, qKp, qKi, m->voltageLimit);
	pi_init(&m->dReg, dKp, dKi, m->voltageLimit);
	pi_init(&m->velReg, velKp, velKi, m->currentLimit);
	pi_setTs(&m->velReg, TS_VEL * 1e-6);
	pi_init(&m->angReg, angKp, 0.0, m->velLimit);
	pi_setTs(&m->velReg, TS_VEL * 1e-6);
	m->open_loop_target = 0;
	m->enabled = 0;
	modbusCommander_writeMotorEnabled(m->enabled);
	modbusCommander_writeMotorTarget(m->target);
	modbusCommander_writeControlMode(m->controlMode);
	modbusCommander_writeCurrentLimit(m->currentLimit);
	modbusCommander_writeVelocityLimit(m->velLimit);
	modbusCommander_writePiGains(qKp, qKi, velKp, velKi, angKp, 0.0);
}

void stepperMotor_linkDriver(stepperMotor *m, stepperDriver2Pwm *d) {
	m->sd2p = d;
	m->voltageLimit = m->sd2p->limit_voltage;
	pi_setLimit(&m->qReg, m->voltageLimit);
	pi_setLimit(&m->dReg, m->voltageLimit);
	modbusCommander_writeVoltageSupply(m->sd2p->supply_voltage);
	modbusCommander_writeVoltageLimit(m->sd2p->limit_voltage);
	stepperDriver2Pwm_disable(m->sd2p);
}

void stepperMotor_linkCurrentSensors(stepperMotor *m, currentSensor *a, currentSensor *b) {
	m->cs_a = a;
	m->cs_b = b;
	modbusCommander_writeCurrentSensorAOffset(m->cs_a->voltage_offset);
	modbusCommander_writeCurrentSensorBOffset(m->cs_b->voltage_offset);
	if (m->cs_a->enabled != 0) currentSensor_stop(m->cs_a);
	if (m->cs_b->enabled != 0) currentSensor_stop(m->cs_b);
}

void stepperMotor_linkAngleSensor(stepperMotor *m, angleSensor *a) {
	m->as = a;
	if (m->cs_a->enabled != 0) angleSensor_stop(m->as);
}

void stepperMotor_setTarget(stepperMotor *m, float target) {
	m->target = target;
}

void stepperMotor_setControlMode(stepperMotor *m, uint8_t mode) {
	// TODO: Check cs, as links
	if (m->controlMode == mode) return;
	if (m->enabled == 1) {
		stepperMotor_disable(m);
		m->controlMode = mode;
		stepperMotor_enable(m);
	} else if(m->enabled == 0) {
		m->controlMode = mode;
	}
}

void stepperMotor_setSupplyVoltage(stepperMotor *m, float supply) {
	if (m->sd2p->supply_voltage == supply) return;
	if (m->enabled == 1) {
		stepperMotor_disable(m);
	}
	m->sd2p->supply_voltage = supply;
}

void stepperMotor_setLimits(stepperMotor *m, float voltage, float current, float velocity) {
	if (m->voltageLimit == voltage &&
			m->currentLimit == current &&
			m->velLimit == velocity) return;
	if (m->enabled == 1) {
		stepperMotor_disable(m);
	}
	m->voltageLimit = voltage;
	pi_setLimit(&m->qReg, m->voltageLimit);
	pi_setLimit(&m->dReg, m->voltageLimit);
	m->sd2p->limit_voltage = voltage;
	m->currentLimit = current;
	pi_setLimit(&m->velReg, current);
	m->velLimit = velocity;
	pi_setLimit(&m->angReg, velocity);
}

void stepperMotor_setCurrentPiGains(stepperMotor *m, float pGain, float iGain) {
	if (m->qReg.P == pGain &&
			m->qReg.P == iGain) return;
	if (m->enabled == 1) {
		stepperMotor_disable(m);
	}
	pi_init(&m->qReg, pGain, iGain, m->voltageLimit);
	pi_init(&m->dReg, pGain, iGain, m->voltageLimit);
}

void stepperMotor_setVelocityPiGains(stepperMotor *m, float pGain, float iGain) {
	if (m->velReg.P == pGain &&
			m->velReg.I == iGain) return;
	if (m->enabled == 1) {
			stepperMotor_disable(m);
	}
	pi_init(&m->velReg, pGain, iGain, m->currentLimit);

}

void stepperMotor_setAnglePiGains(stepperMotor *m, float pGain, float iGain) {
	if (m->angReg.P == pGain &&
			m->angReg.I == iGain) return;
	if (m->enabled == 1) {
			stepperMotor_disable(m);
	}
	pi_init(&m->angReg, pGain, iGain, m->velLimit);
}

void stepperMotor_enable(stepperMotor *m) {
	if (m->cs_a != NULL) {
		currentSensor_start(m->cs_a);
	}
	if (m->cs_b != NULL) {
		currentSensor_start(m->cs_b);
	}
	if (m->as != NULL) {
	angleSensor_start(m->as);
	}
	m->enabled = 1;
}

void stepperMotor_disable(stepperMotor *m) {
//	if (m->cs_a != NULL) {
//		currentSensor_stop(m->cs_a);
//	}
//	if (m->cs_b != NULL) {
//		currentSensor_stop(m->cs_b);
//	}
//	if (m->as != NULL) {
//		angleSensor_stop(m->as);
//	}
	if (m->sd2p != NULL) {
		stepperDriver2Pwm_disable(m->sd2p);
	}
	m->enabled = 0;
}

void stepperMotor_targetLoop(stepperMotor *m) {
	if (m->enabled != 1) return;
	if (m->sd2p->pwm_timer->Instance->CCR1 == 0) m->cs_a->raw_value[0] = 0;
	if (m->sd2p->pwm_timer->Instance->CCR2 == 0) m->cs_b->raw_value[0] = 0;
	switch (m->controlMode) {
	case TORQUE_CONTROL:
		m->currentDes = m->target;
		break;
	case ANGLE_OPEN_LOOP:
		stepperMotor_openLoopAngleMode(m, m->target);
		break;
	case VOLTAGE_TEST_ALPHA_PHASE:
		stepperMotor_voltageTestModeAphase(m, m->target);
		break;
	case VOLTAGE_TEST_BETA_PHASE:
		stepperMotor_voltageTestModeBphase(m, m->target);
		break;
	default:
		return;
	}
}

void stepperMotor_focCurrentLoop(stepperMotor *m) {
	if (m->enabled != 1) return;
	if (m->controlMode == ANGLE_OPEN_LOOP || m->controlMode == VOLTAGE_TEST_BETA_PHASE ||
			m->controlMode == VOLTAGE_TEST_ALPHA_PHASE) return;
	m->shaftAngle = angleSensor_getAngle(m->as);
	float el_angle = electricalAngle(m->shaftAngle, m->polePairs, m->zeroElectricalAngle);
	getDqCurrents(m, el_angle);
	// TODO: filtering m->current
	m->dqVoltage.q = pi_activate(&m->qReg, (m->currentDes - m->dqCurrent.q));
	m->dqVoltage.d = pi_activate(&m->dReg, m->dqCurrent.d);
	setPhaseVoltage(m, m->dqVoltage.q, m->dqVoltage.d, el_angle);
}

void stepperMotor_openLoopAngleMode(stepperMotor *m, float target_angle) {
	float ts = TS_VEL * 1e-6;
	if (ABS(target_angle - m->shaftAngle) > ABS(m->velLimit * ts)) {
		m->shaftAngle += SIGN(target_angle - m->shaftAngle) * ABS(m->velLimit) * ts;
	} else {
		m->shaftAngle = target_angle;
		stepperDriver2Pwm_disable(m->sd2p);
		return;
	}

	float Uq = m->sd2p->limit_voltage;
	setPhaseVoltage(m, Uq, 0, electricalAngle(m->shaftAngle, m->polePairs, m->zeroElectricalAngle));
}

void stepperMotor_voltageTestModeAphase(stepperMotor *m, float target_volt) {
	m->abVoltage.a = target_volt;
	m->abVoltage.b = 0;
	stepperDriver2Pwm_setPwm(m->sd2p, m->abVoltage.a, m->abVoltage.b);
}

void stepperMotor_voltageTestModeBphase(stepperMotor *m, float target_volt) {
	m->abVoltage.a = 0;
	m->abVoltage.b = target_volt;
	stepperDriver2Pwm_setPwm(m->sd2p, m->abVoltage.a, m->abVoltage.b);
}

void setPhaseVoltage(stepperMotor *m, float Uq, float Ud, float angle_el) {
	if (m->sd2p->is_init != 1) return;
	// Sinusoidal PWM modulation
	// Inverse Park transformation
	double _ca = cos(angle_el);
	double _sa = sin(angle_el);
	// Inverse park transform
	m->abVoltage.a =  _ca * Ud - _sa * Uq;  // -sin(angle) * Uq;
	m->abVoltage.b =  _sa * Ud + _ca * Uq;    //  cos(angle) * Uq;
	stepperDriver2Pwm_setPwm(m->sd2p, m->abVoltage.a, m->abVoltage.b);
}

void getDqCurrents(stepperMotor* m, float angle_el) {
	if (m->cs_a->enabled == 1 && m->cs_b->enabled == 1) {
		m->abCurrent.a = currentSensor_getCurrent(m->cs_a);
		m->abCurrent.b = currentSensor_getCurrent(m->cs_b);
		// calculate park transform
		float ct = cos(angle_el);
		float st = sin(angle_el);
		m->dqCurrent.d = m->abCurrent.a * ct + m->abCurrent.b * st;
		m->dqCurrent.q = m->abCurrent.b * ct - m->abCurrent.a * st;
	}
}

void stepperMotor_electricalAngleCalibrate(stepperMotor *m) {
	stepperMotor_disable(m);
	angleSensor_start(m->as);
	float Ua = 0.0f;
	float Ub = 0.0f;
	Ua = sequencerFullStep[0][0] * (m->voltageLimit);
	Ub = sequencerFullStep[1][0] * (m->voltageLimit);
	stepperDriver2Pwm_setPwm(m->sd2p, Ua, Ub);
	HAL_Delay(100);
	m->zeroElectricalAngle = electricalAngle(angleSensor_getAngle(m->as), m->polePairs, 0.0);
	modbusCommander_writeZeroAngleOffset(m->zeroElectricalAngle);
//	for (int step = 0; step < m->polePairs; step++) {
//		Ua = sequencerFullStep[0][step % 4] * (m->voltageLimit / 2);
//		Ub = sequencerFullStep[1][step % 4] * (m->voltageLimit / 2);
//		stepperDriver2Pwm_setPwm(m->sd2p, Ua, Ub);
//		if ((step % 4) == 0) {
//			Ua = sequencerFullStep[0][step % 4] * (m->voltageLimit);
//			Ub = sequencerFullStep[1][step % 4] * (m->voltageLimit);
//			stepperDriver2Pwm_setPwm(m->sd2p, Ua, Ub);
//			HAL_Delay(50);
//			angle = angleSensor_getAngle(m->as);
//			angleDeg = angle * (180.0 / 3.141592);
//		}
//		HAL_Delay(50);
//	}
	stepperDriver2Pwm_disable(m->sd2p);
	angleSensor_stop(m->as);
}


void stepperMotor_angleSensorLoop(stepperMotor *m) {
	angleSensor_velocityMeasurementLoop(m->as);
}


