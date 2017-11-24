/*
 * pid_attitude_controller.h
 *
 *  Created on: 15 nov. 2017
 *      Author: diego.aguirre
 */

#ifndef _PID_ATTITUDE_CONTROLLER_H_
#define _PID_ATTITUDE_CONTROLLER_H_

#include "../utils/common.h"
#include "pid.h"

#define PID_ROLL_RATE_KP  250.0f
#define PID_ROLL_RATE_KI  500.0f
#define PID_ROLL_RATE_KD  2.5f
#define PID_ROLL_RATE_INTEGRATION_LIMIT    33.3f

#define PID_PITCH_RATE_KP  250.0f
#define PID_PITCH_RATE_KI  500.0f
#define PID_PITCH_RATE_KD  2.5f
#define PID_PITCH_RATE_INTEGRATION_LIMIT   33.3f

#define PID_YAW_RATE_KP  120.0f
#define PID_YAW_RATE_KI  16.7f
#define PID_YAW_RATE_KD  0.0f
#define PID_YAW_RATE_INTEGRATION_LIMIT     166.7f

#define PID_ROLL_KP  6.0f
#define PID_ROLL_KI  3.0f
#define PID_ROLL_KD  0.0f
#define PID_ROLL_INTEGRATION_LIMIT    20.0f

#define PID_PITCH_KP  6.0f
#define PID_PITCH_KI  3.0f
#define PID_PITCH_KD  0.0f
#define PID_PITCH_INTEGRATION_LIMIT   20.0f

#define PID_YAW_KP  6.0f
#define PID_YAW_KI  1.0f
#define PID_YAW_KD  0.35f
#define PID_YAW_INTEGRATION_LIMIT     360.0f

/**
 * PID controller initialization.
 *
 * @param[in] dt  update period of attitude controller in seconds.
 */
void init_attitude_controller(const float dt);

/**
 * Attitude controller update to correct actual value and reduce error with set point(desired value)
 *
 * @param[in]  actual_roll_value
 * @param[in]  actual_pitch_value
 * @param[in]  actual_yaw_value
 * @param[in]  desired_roll_value 		roll set point for pid
 * @param[in]  desired_pitch_value      pitch set point for pid
 * @param[in]  desired_yaw_value        yaw set point for pid
 * @param[out] roll_output        		roll output returned by pid controller
 * @param[out] pitch_output 			pitch output returned by pid controller
 * @param[out] yaw_output 				yaw output returned by pid controller
 */
void attitude_controller_update(madgwick_DoF_output_t madgwick_DoF_output, const rpy_t current_attitude_values, const rpy_t desired_attitude_values,
		rpy_t *output_rpy_values);

/**
 * rate controller update to correct actual value and reduce error with set point(desired value)
 *
 * @param[in]  actual_roll_value
 * @param[in]  actual_pitch_value
 * @param[in]  actual_yaw_value
 * @param[in]  desired_roll_rate 		roll set point for pid
 * @param[in]  desired_pitch_rate      pitch set point for pid
 * @param[in]  desired_yaw_rate        yaw set point for pid
 * @param[out] roll_output        		roll output returned by pid controller
 * @param[out] pitch_output 			pitch output returned by pid controller
 * @param[out] yaw_output 				yaw output returned by pid controller
 */
void rate_controller_update(madgwick_DoF_output_t madgwick_DoF_output, const rpy_t current_attitude_rate,
		const rpy_t desired_attitude_rate, rpy_t *output_rpy_rates);


#endif /* MODULES_PID_ATTITUDE_CONTROLLER_H_ */
