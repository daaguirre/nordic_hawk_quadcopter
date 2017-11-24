/*
 * nordic_hawk_stabilizer.c
 *
 *  Created on: 15 nov. 2017
 *      Author: diego.aguirre
 */

#include "quadcopter_stabilizer.h"
#include "sensor_fusion.h"
#include "pid_attitude_controller.h"
#include "common.h"
#include "quadcopter_config.h"
#include "nordic_hawk_pwm_motor.h"

quadcopter_attitude_t current_nordic_hawk_attitude;
quadcopter_attitude_t desired_nordic_hawk_attitude;
rpy_t stabilizer_pid_output;
const float base_motor_thrust = 675.0f;

static void map_pid_output_on_motors(const rpy_t pid_output);
static int limit_thrust(float unlimited_thrust);

void init_stabilizer()
{
	sensor_fusion_init();
	init_attitude_controller(1);
	desired_nordic_hawk_attitude.attitude.roll = 0.0f;
	desired_nordic_hawk_attitude.attitude.pitch = 0.0f;
	desired_nordic_hawk_attitude.attitude.yaw = 0.0f;
}

void stabilizer_update()
{
	madgwick_DoF_output_t madgwick_DoF_output = sensor_fusion_update(&current_nordic_hawk_attitude);

	attitude_controller_update(madgwick_DoF_output, current_nordic_hawk_attitude.attitude,
								desired_nordic_hawk_attitude.attitude, &desired_nordic_hawk_attitude.attitude_rate);
	rate_controller_update(madgwick_DoF_output, current_nordic_hawk_attitude.attitude_rate, desired_nordic_hawk_attitude.attitude_rate,
							&stabilizer_pid_output);

	map_pid_output_on_motors(stabilizer_pid_output);
}

static void map_pid_output_on_motors(const rpy_t pid_output)
{
	int motor1 = limit_thrust(base_motor_thrust - pid_output.roll + pid_output.pitch + pid_output.yaw);
	int motor2 = limit_thrust(base_motor_thrust - pid_output.roll - pid_output.pitch - pid_output.yaw);
	int motor3 = limit_thrust(base_motor_thrust + pid_output.roll - pid_output.pitch + pid_output.yaw);
	int motor4 = limit_thrust(base_motor_thrust + pid_output.roll + pid_output.pitch - pid_output.yaw);

	update_motor_values(motor1, motor2, motor3, motor4);
}

static int limit_thrust(float unlimited_thrust)
{
	int motor_output;
	if(unlimited_thrust > MAX_MOTOR_THRUST)
	{
		return motor_output = MAX_MOTOR_THRUST;
	}

	if(unlimited_thrust < MIN_MOTOR_THRUST)
	{
		return motor_output = MIN_MOTOR_THRUST;
	}

	return motor_output = (int)unlimited_thrust;
}
