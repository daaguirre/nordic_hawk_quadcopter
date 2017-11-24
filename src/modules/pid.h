/*
 * pid.h
 *
 *  Created on: 13 nov. 2017
 *      Author: diego.aguirre
 */

#ifndef _PID_H_
#define _PID_H_

#include <stdbool.h>

#define DEFAULT_PID_INTEGRATION_LIMIT 		5000.0f
#define DEFAULT_PID_OUTPUT_LIMIT      		0.0f

typedef struct{
	float error;
	float set_point; //desired value
	float prev_error;//previous error
	float integral;
	float derivative;
	float kp; //proportional gain
	float ki; //integral gain
	float kd; //derivative gain
	float out_p; //proportional output
	float out_i; //integral output
	float out_d; //derivative output
	float i_limit; //integral limit
	float output_limit;
	float dt; //delta time
}pid_controller_t;

/**
 * PID controller initialization.
 *
 * @param[out] pid   A pointer to the pid controller to initialize.
 * @param[in] desired_value  The initial set point.
 * @param[in] kp        The proportional gain
 * @param[in] ki        The integral gain
 * @param[in] kd        The derivative gain
 * @param[in] dt        Delta time since the last call
 * @param[in] samplingRate Frequency the update will be called
 */
void pid_init(pid_controller_t *pid_controller, float desired_value, float kp, float ki,
				float kd, float dt);

/**
 * Update the PID parameters.
 *
 * @param[in] pid         A pointer to the pid object.
 * @param[in] measured    The measured value
 * @param[in] updateError Set to TRUE if error should be calculated.
 *                        Set to False if pidSetError() has been used.
 * @return PID algorithm output
 */
float pid_update(pid_controller_t* pid, const float measured, const bool updateError);

/**
 * Reset the PID error values
 *
 * @param[in] pid   A pointer to the pid controller.
 */
void pid_controller_reset(pid_controller_t *pid);

#endif /* MODULES_PID_H_ */
