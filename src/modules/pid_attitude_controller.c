/*
 * pid_attitude_controller.c
 *
 *  Created on: 15 nov. 2017
 *      Author: diego.aguirre
 */


#include "pid_attitude_controller.h"
#include "pid.h"
#include "common.h"
#include <stdbool.h>

pid_controller_t pid_roll;
pid_controller_t pid_pitch;
pid_controller_t pid_yaw;

pid_controller_t pid_roll_rate;
pid_controller_t pid_pitch_rate;
pid_controller_t pid_yaw_rate;

void init_attitude_controller(const float dt)
{
	pid_init(&pid_roll, 0, PID_ROLL_KP , PID_ROLL_KI, PID_ROLL_KD, dt);
	pid_init(&pid_pitch, 0, PID_PITCH_KP , PID_PITCH_KI, PID_PITCH_KD, dt);
	pid_init(&pid_yaw, 0, PID_YAW_KP , PID_YAW_KI, PID_YAW_KD, dt);

	pid_roll.i_limit = PID_ROLL_INTEGRATION_LIMIT;
	pid_pitch.i_limit = PID_PITCH_INTEGRATION_LIMIT;
	pid_yaw.i_limit = PID_YAW_INTEGRATION_LIMIT;

	pid_init(&pid_roll_rate, 0, PID_ROLL_RATE_KP , PID_ROLL_RATE_KI, PID_ROLL_RATE_KD, dt);
	pid_init(&pid_pitch_rate, 0, PID_PITCH_RATE_KP , PID_PITCH_RATE_KI, PID_PITCH_RATE_KD, dt);
	pid_init(&pid_yaw_rate, 0, PID_YAW_RATE_KP , PID_YAW_RATE_KI, PID_YAW_RATE_KD, dt);

	pid_roll.i_limit = PID_ROLL_RATE_INTEGRATION_LIMIT;
	pid_pitch.i_limit = PID_PITCH_RATE_INTEGRATION_LIMIT;
	pid_yaw.i_limit = PID_YAW_RATE_INTEGRATION_LIMIT;
}

/**
 * Attitude controller update to correct actual value and reduce error with set point(desired value)
 *
 * @param[in]  madgwick_DoF_output	    Degrees of freedom in madgwick algorithm output. YAW PID output can
 * 										only be calculated with 9DoF
 * @param[in]  current_attitude_rate	roll, pitch, yaw values
 * @param[in]  desired_attitude_rate	roll, pitch, yaw set points for pid
 * @param[out] output_rpy_rates 	    roll, pitch, yaw pid output.
 */
void attitude_controller_update(madgwick_DoF_output_t madgwick_DoF_output, const rpy_t current_attitude_values,
		const rpy_t desired_attitude_values, rpy_t *output_rpy_values)
{
	//update desired values
	pid_roll.set_point = desired_attitude_values.roll;
	pid_pitch.set_point = desired_attitude_values.pitch;

	output_rpy_values->roll = pid_update(&pid_roll, current_attitude_values.roll, true);
	output_rpy_values->pitch = pid_update(&pid_pitch, current_attitude_values.pitch, true);

#ifdef CONSTRAINED_YAW_PID
	if(madgwick_DoF_output == madgwick_9DoF)
	{
		pid_yaw.set_point = desired_attitude_values.yaw;
		output_rpy_values->yaw = pid_update(&pid_yaw, current_attitude_values.yaw, true);
	}
#else
	pid_yaw.set_point = desired_attitude_values.yaw;
	output_rpy_values->yaw = pid_update(&pid_yaw, current_attitude_values.yaw, true);
#endif

}

/**
 * rate controller update to correct actual value and reduce error with set point(desired value).
 *
 * @param[in]  madgwick_DoF_output	    Degrees of freedom in madgwick algorithm output. YAW PID output can
 * 										only be calculated with 9DoF
 * @param[in]  current_attitude_rate	roll, pitch, yaw values
 * @param[in]  desired_attitude_rate	roll, pitch, yaw set points for pid
 * @param[out] output_rpy_rates 	    roll, pitch, yaw pid output.
 */
void rate_controller_update(madgwick_DoF_output_t madgwick_DoF_output, const rpy_t current_attitude_rate,
		const rpy_t desired_attitude_rate, rpy_t *output_rpy_rates)
{
	//update desired values
	pid_roll.set_point = desired_attitude_rate.roll;
	pid_pitch.set_point = desired_attitude_rate.pitch;

	output_rpy_rates->roll = pid_update(&pid_roll, current_attitude_rate.roll, true);
	output_rpy_rates->pitch = pid_update(&pid_pitch, current_attitude_rate.pitch, true);

#ifdef CONSTRAINED_YAW_PID
	if(madgwick_DoF_output == madgwick_9DoF)
	{
		pid_yaw.set_point = desired_attitude_rate.yaw;
		output_rpy_rates->yaw = pid_update(&pid_yaw, current_attitude_rate.yaw, true);
	}
#else
	pid_yaw.set_point = desired_attitude_rate.yaw;
	output_rpy_rates->yaw = pid_update(&pid_yaw, current_attitude_rate.yaw, true);
#endif
}
