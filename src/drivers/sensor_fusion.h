/*
 * sensor_fusion.h
 *
 *  Created on: 13 sept. 2017
 *      Author: Diego Aguirre
 */

#ifndef _SENSOR_FUSION_H_
#define _SENSOR_FUSION_H_

#include <stdbool.h>

#include "../utils/common.h"

typedef struct quadcopter_attitude_t{
	rpy_t attitude;
	rpy_t attitude_rate;
}quadcopter_attitude_t;

bool sensor_fusion_init();
bool mpu_meas_read(void * p_context);
madgwick_DoF_output_t sensor_fusion_update(quadcopter_attitude_t *quadcopter_attitude);
void get_euler_angles(float *euler_angles_array);

#endif /* DRIVERS_SENSOR_FUSION_H_ */
