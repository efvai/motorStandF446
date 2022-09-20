/*
 * stepperMotor.h
 *
 *  Created on: 28 июл. 2022 г.
 *      Author: jkcvk
 */

#ifndef INC_STEPPERMOTOR_H_
#define INC_STEPPERMOTOR_H_

#include "stepperDriver2Pwm.h"
#include "currentSensor.h"
#include "angleSensor.h"

#define SIGN(a) ( ( (a) < 0 )  ?  -1   : ( (a) > 0 ) )

// MODES
#define TORQUE_CONTROL 0
#define VEL_CONTROL 1
#define ANGLE_CONTROL 2
#define ANGLE_OPEN_LOOP 3
#define VOLTAGE_TEST_ALPHA_PHASE 4
#define VOLTAGE_TEST_BETA_PHASE 5

// DEFAULT PI PARAMS
#define qKp 3
#define qKi 300.0f
#define dKp 3
#define dKi 300.0f
#define velKp 1
#define velKi 10
#define angKp 20
#define defvellim 5

// Time samples (microseconds)
#define TS_VEL 1000 // Velocity loop (1000 Hz)
#define TS_CUR 100 // Current loop (10000 Hz)

typedef struct {
	float q;
	float d;
} dq;

typedef struct {
	float a;
	float b;
} phase;

typedef struct {
	float P;
	float I;
	float errorPrev;
	float integralPrev;
	float limit;
	float ts;
} pi_reg;

typedef struct {
	float ts;
	float y_prev;
	float time_constant;
} low_pass_filter;

typedef struct {
	stepperDriver2Pwm *sd2p;
	currentSensor *cs_a;
	currentSensor *cs_b;
	angleSensor *as;

	uint8_t polePairs;
	dq dqCurrent;
	dq dqVoltage;
	phase abCurrent;
	phase abVoltage;
	float currentDes;
	float target;

	float shaftAngle;

	//limits
	float currentLimit;
	float voltageLimit;
	float velLimit;

	//controls
	uint16_t controlMode;
	uint8_t enabled;
	//pids
	pi_reg qReg;
	pi_reg dReg;
	pi_reg velReg;
	pi_reg angReg;


	int open_loop_target;
} stepperMotor;


void pi_init(pi_reg*, float p, float i, float limit);
void pi_setTs(pi_reg *, float ts);
void pi_setLimit(pi_reg *, float lim);
float pi_activate(pi_reg*, float);

void lowPassFilter_init(low_pass_filter *lpf, float tf, float ts);
float lowPassFilter_activate(low_pass_filter *lpf, float x);

void stepperMotor_init(stepperMotor *m);
void stepperMotor_linkDriver(stepperMotor *m, stepperDriver2Pwm *d);
void stepperMotor_linkCurrentSensors(stepperMotor *m, currentSensor *a, currentSensor *b);
void stepperMotor_linkAngleSensor(stepperMotor *m, angleSensor *a);
void stepperMotor_shaftAngleCalibration(stepperMotor *m);

void stepperMotor_setTarget(stepperMotor *m, float target);
void stepperMotor_setControlMode(stepperMotor *m, uint8_t mode);
void stepperMotor_setLimits(stepperMotor *m, float, float, float);
void stepperMotor_setSupplyVoltage(stepperMotor *m, float);
void stepperMotor_setCurrentPiGains(stepperMotor *m, float, float);
void stepperMotor_setVelocityPiGains(stepperMotor *m, float, float);
void stepperMotor_setAnglePiGains(stepperMotor *m, float, float);
void stepperMotor_enable(stepperMotor *m);
void stepperMotor_disable(stepperMotor *m);

void stepperMotor_openLoopFullStepMode(stepperMotor *m, float target);
void stepperMotor_openLoopAngleMode(stepperMotor *m, float target_angle);
void stepperMotor_voltageTestModeAphase(stepperMotor *m, float target_volt);
void stepperMotor_voltageTestModeBphase(stepperMotor *m, float target_volt);

void stepperMotor_targetLoop(stepperMotor *m);
void stepperMotor_focCurrentLoop(stepperMotor *m);
void stepperMotor_angleSensorLoop(stepperMotor *m);

//foc things
void setPhaseVoltage(stepperMotor *m, float Uq, float Ud, float angle_el);
void getDqCurrents(stepperMotor* m, float angle_el);
float electricalAngle(float mech_angle, int pole_pairs);

#endif /* INC_STEPPERMOTOR_H_ */
