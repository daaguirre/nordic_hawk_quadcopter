/*
 * quadcopter.h
 *
 *  Created on: 22 nov. 2017
 *      Author: diego.aguirre
 */

#ifndef UTILS_COMMON_H_
#define UTILS_COMMON_H_

typedef struct attitude_state_t {
	float roll;
	float pitch;
	float yaw;
}rpy_t;

typedef enum{
	madgwick_6DoF,
	madgwick_9DoF,
}madgwick_DoF_output_t;

#endif /* UTILS_COMMON_H_ */
