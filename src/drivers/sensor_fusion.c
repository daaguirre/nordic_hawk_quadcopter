/*
 * sensor_fusion.c
 *
 *  Created on: 13 sept. 2017
 *      Author: Diego Aguirre
 */

#include <common_utils.h>
#include <math.h>
#include <stdlib.h>
#include <stdio.h>
#include "mpu9250.h"
#include "sensor_fusion.h"
#include "real_time_debugger.h"
#include "app_timer.h"
#include "ble_sensor_service.h"
#include "MadgwickAHRS.h"
#include "quadcopter_config.h"
#include <string.h>

axis_float_t accel_meas;
axis_float_t gyro_meas;
axis_float_t mag_meas;

static rpy_t calculate_euler_angles(float qt0, float qt1, float qt2, float qt3, madgwick_DoF_output_t madgwick_DoF_output);

//APP_TIMER_DEF(m_mpu_timer_read);

//MPU9250 Configuration for nordic hawk quadcopter
const mpu9250_config_t nordic_hawk_config = {
		.accel_fsr = MPU9250_ACCEL_FS_4,
		.gyro_fsr = MPU9250_GYRO_FS_500,
		.mag_fsr = MPU9250_MAG_FS_16,
		.gyro_fchoice = 0x03,  //LPF enabled. BW = 250Hz && delay = 0.95ms
		.accel_fchoice = 0x01, //LPF enabled. BW = 218.1 Hz && delay = 1.88ms
		.gyro_temp_dlpf = 0x00, //LPF enabled. BW = 250Hz && delay = 0.95ms
		.accel_dlpf = 0x00, //LPF enabled. BW = 218.1 Hz && delay = 1.88s
		.mag_operating_mode = 0x06, //continuous mode 2 (100Hz)
		.fifo_enable = 0x00,
		.int_enable = 0x00,
		.sample_rate_div = 0x00
};

/**
 *  @brief  Sensor fusion initialization.
 *  Initializes all the sensors needed to implent a specific
 *  sensor fusion algorithm (Madckwick or Mahony)
 *
 *  @return True if succed, false otherwise
 */
bool sensor_fusion_init()
{

	if(!mpu9250_init(&nordic_hawk_config))
		return false;

#ifdef DEBUG
	rtt_println("All sensors initialized succesfully\n");
#endif

//    // Create battery timer.
//	ret_code_t err_code = app_timer_create(&m_mpu_timer_read,
//                                APP_TIMER_MODE_REPEATED,
//								mpu_meas_read_timeout_handler);
//    APP_ERROR_CHECK(err_code);

	return true;
}

madgwick_DoF_output_t sensor_fusion_update(quadcopter_attitude_t *quadcopter_attitude)
{
	float quaternion[4] = {0.0f, 0.0f, 0.0f, 0.0f};
	madgwick_DoF_output_t madgwick_DoF_output;
	mpu9250_get_accel_data_gs(&accel_meas);
	mpu9250_get_gyro_data_rps(&gyro_meas);
	mpu9250_get_mag_data_miligauss(&mag_meas);

	quadcopter_attitude->attitude_rate.roll = gyro_meas.x;
	quadcopter_attitude->attitude_rate.pitch = gyro_meas.y;
	quadcopter_attitude->attitude_rate.yaw = gyro_meas.z;

//	rtt_print_float("gx: ", gyro_meas.x);
//	rtt_print_float("gy: ", gyro_meas.y);
//	rtt_print_float("gz: ", gyro_meas.z);
//	uint32_t old_ticks = app_timer_cnt_get();


//	for(uint8_t i = 0; i < MADGWICK_ITERATIONS; i++)
//	{
		madgwick_DoF_output =  MadgwickAHRSupdate(gyro_meas.x, gyro_meas.y, gyro_meas.z,
							accel_meas.x, accel_meas.y, accel_meas.z,
							mag_meas.y, mag_meas.x, -mag_meas.z);
//	}
//	uint32_t current_ticks = app_timer_cnt_get();
//	uint32_t ticks_elapsed = app_timer_cnt_diff_compute(current_ticks, old_ticks);
//	rtt_print_uint32("elapsed:", ticks_elapsed);
	get_rotation_quaternion(quaternion);

	//conjugate quaternion because quaternion obtained from algorimth
	// is relative to sensor frame
	quadcopter_attitude->attitude = calculate_euler_angles(quaternion[0], -quaternion[1], -quaternion[2], -quaternion[3], madgwick_DoF_output);

	return madgwick_DoF_output;
}


static rpy_t calculate_euler_angles(float q0, float q1, float q2, float q3, madgwick_DoF_output_t madgwick_DoF_output)
{
	rpy_t euler_angles;

	float roll_y = 2*(q2*q3 - q0*q1);
	float roll_x = 2*(q0*q0 + q3*q3) - 1;
	euler_angles.roll = atan2f(roll_y, roll_x);

	float pitch_x = 2*(q1*q3 + q0*q2);
	euler_angles.pitch = -atanf(pitch_x/sqrt(1-pitch_x*pitch_x));

	if(madgwick_DoF_output == madgwick_9DoF)
	{
		float yaw_y = 2*(q1*q2-q0*q3);
		float yaw_x = 2*(q0*q0 + q1*q1) - 1;
		euler_angles.yaw = atan2f(yaw_y, yaw_x);
		euler_angles.yaw = euler_angles.yaw * 180/PI - 0.8;
	}
	else
	{
		euler_angles.yaw = 0.0f;
	}

	euler_angles.roll = euler_angles.roll * 180/PI;
	euler_angles.pitch = euler_angles.pitch * 180/PI;

#ifdef SENSOR_DEBUG
	if(madgwick_DoF_output == madgwick_9DoF)
	{
		rtt_print_float("roll: ", euler_angles.roll);
		rtt_print_float("pitch: ", euler_angles.pitch);
		rtt_print_float("yaw: ", euler_angles.yaw);
	}
#endif

	return euler_angles;

}

//bool mpu_meas_read(void * p_context)
//{
//    UNUSED_PARAMETER(p_context);
//
//    int16_t accel_meas[3] = {0, 0, 0};
//
//    bool res = mpu9250_read_accel_data(accel_meas);
//    if(!res)
//    {
//    	return false;
//    }
//
//    return true;
//}


