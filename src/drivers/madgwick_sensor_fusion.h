/*
 * madgwick_sensor_fusion.h
 *
 *  Created on: 6 oct. 2017
 *      Author: diego.aguirre
 */

#ifndef DRIVERS_MADGWICK_SENSOR_FUSION_H_
#define DRIVERS_MADGWICK_SENSOR_FUSION_H_

void madgwick_filter_update(float a_x, float a_y, float a_z, float w_x, float w_y, float w_z, float m_x, float m_y, float m_z, float *euler_angles);

#endif /* DRIVERS_MADGWICK_SENSOR_FUSION_H_ */
