/*
 * utils.h
 *
 *  Created on: Sep 15, 2022
 *      Author: user
 */

#ifndef INC_UTILS_H_
#define INC_UTILS_H_

#define _PI 3.14159265359f
#define _PI_2 1.57079632679f
#define _PI_3 1.0471975512f
#define _2PI 6.28318530718f
#define _3PI_2 4.71238898038f
#define _PI_6 0.52359877559f


#define SIGN(a) ( ( (a) < 0 )  ?  -1   : ( (a) > 0 ) )

typedef struct {
	float q;
	float d;
} dq;

typedef struct {
	float a;
	float b;
} phase;

typedef union  {
    uint16_t* raw;
    float* value;
} param_float;

typedef union {
	uint16_t* raw;
	int *value;
} param_int;

float normalizeAngle(float angle);

float electricalAngle(float mech_angle, int pole_pairs, float zeroElectricalAngle);


#endif /* INC_UTILS_H_ */
