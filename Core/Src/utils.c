/*
 * utils.c
 *
 *  Created on: Sep 15, 2022
 *      Author: user
 */

#include "utils.h"
#include <math.h>

float normalizeAngle(float angle) {
	  float a = fmod(angle, _2PI);
	  return a >= 0 ? a : (a + _2PI);
}

float electricalAngle(float mech_angle, int pole_pairs, float zero_el) {
	return normalizeAngle((mech_angle * pole_pairs) - zero_el);
}
