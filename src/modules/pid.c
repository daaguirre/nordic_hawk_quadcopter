/*
 * pid.c
 *
 *  Created on: 13 nov. 2017
 *      Author: diego.aguirre
 */

#include <common_utils.h>
#include "pid.h"

void pid_init(pid_controller_t *pid, const float desired_value, const float kp, const float ki,
				const float kd, const float dt)
{
	pid->error = 0.0f;
	pid->prev_error = 0.0f;
	pid->integral = 0.0f;
	pid->derivative = 0.0f;
	pid->set_point = desired_value;
	pid->kp = kp;
	pid->ki = ki;
	pid->kd = kd;
	pid->i_limit = DEFAULT_PID_INTEGRATION_LIMIT;
	pid->output_limit = DEFAULT_PID_OUTPUT_LIMIT;
	pid->dt = dt;
}

float pid_update(pid_controller_t* pid, const float measured, const bool updateError)
{
    float output = 0.0f;

    if (updateError)
    {
        pid->error = pid->set_point - measured;
    }

    pid->out_p = pid->kp * pid->error;
    output += pid->out_p;

    pid->derivative = (pid->error - pid->prev_error) / pid->dt;
//    if (pid->enableDFilter)
//    {
//      pid->deriv = lpf2pApply(&pid->dFilter, deriv);
//    } else {
//      pid->deriv = deriv;
//    }
//    if (isnan(pid->deriv)) {
//      pid->deriv = 0;
//    }
    pid->out_d = pid->kd * pid->derivative;
    output += pid->out_d;

    pid->integral += pid->error * pid->dt;

    // Constrain the integral (unless the iLimit is zero)
    if(pid->i_limit != 0)
    {
    	pid->integral = constrain(pid->integral, -pid->i_limit, pid->i_limit);
    }

    pid->out_i = pid->ki * pid->integral;
    output += pid->out_i;

    // Constrain the total PID output (unless the outputLimit is zero)
    if(pid->output_limit != 0)
    {
      output = constrain(output, -pid->output_limit, pid->output_limit);
    }


    pid->prev_error = pid->error;

    return output;
}

void pid_controller_reset(pid_controller_t *pid)
{
	pid->error = 0.0f;
	pid->prev_error = 0.0f;
	pid->integral = 0.0f;
	pid->derivative = 0.0f;
}
