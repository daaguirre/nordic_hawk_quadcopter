/*
 * mpu9250.c
 *
 *  Created on: 23 ago. 2017
 *      Author: Diego Aguirre
 */

#include <common_utils.h>
#include <math.h>
#include "i2cdev.h"
#include <stdlib.h>
#include <stdio.h>
#include "nrf_delay.h"
#include "mpu9250.h"

#ifdef DEBUG
#include "real_time_debugger.h"
#endif

static void mpu9250_set_accel_res();
static void mpu9250_set_gyro_res();
static void mpu9250_set_mag_res();
static void mpu9250_set_accel_bias_reg(int32_t *accel_bias);
static void mpu9250_set_gyro_bias_reg(int32_t *gyro_bias);
static void mpu9250_config_cal();
static void mpu9250_calculate_bias(int32_t *accel_bias, int32_t *gyro_bias);
static void mpu9250_get_mag_cal_data();
#ifdef MAG_CALIBRATION
static void mpu9250_get_mag_bias();
#endif
static bool mpu9250_evaluate_self_test(float low, float high, float value, char* string);

static mpu9250_t mpu9250 = {
	.dev_addr = MPU9250_I2C_ADDRESS,
	.mag_addr = AK8963_ADDRESS,
};

/**
 *  @brief  MPU9250 Initialization:
 *  		- Accel & Gyro test connection
 *  		- Accel & Gyro self test
 *  		- Accel & Gyro bias calibration
 *  		- Set configuration for Accel & Gyro and active bypass to
 *  		stablish direct communication with AK8963 compass
 *  		- AK8963 magnetometer test connection
 *  		- Get factory calibration data and bias for magnetometer
 *  		- Set indicated configuration for magnetometer
 *
 *  @return true if successful, otherwise false
 */
bool mpu9250_init(const mpu9250_config_t *device_config)
{
	mpu9250.config_data = device_config;
	mpu9250_set_accel_res();
	mpu9250_set_gyro_res();
	mpu9250_set_mag_res();

  	if(!mpu9250_test_connection())
  		return false;

  	if(mpu9250_accel_gyro_self_test() != 0x03)
  		return false;

  	mpu9250_accel_gyro_bias_cal();

  	mpu9250_set_config();

  	if(!mpu9250_mag_test_connection())
  		return false;

  	mpu9250_init_mag();
  	mpu9250_set_mag_config();


  	return true;
}

/**
 *  @brief  Initialize magnetomer of MPU9250 and set bias and calibration data
 *
 *  @return None
 */
void mpu9250_init_mag()
{
	mpu9250_get_mag_cal_data();

#ifdef MAG_CALIBRATION
	mpu9250_get_mag_bias();
#else
	mpu9250_set_mag_cal_data();
#endif

}

/** Verify the I2C connection with mpu6500 and magnetometer
 * Make sure the device is connected and responds as expected.
 * @return True if connection is valid, false otherwise
 */
bool mpu9250_test_connection(void)
{
	if(!(mpu9250_get_whoami() == 0x71))//0x71 is MPU9250 ID with AD0 = 0;
		return false;

#ifdef DEBUG
	rtt_println("MPU9250 connected!\n");
#endif

   return true;
}

bool mpu9250_mag_test_connection()
{
	if(!(mpu9250_get_mag_whoami() == 0x48))
			return false;

#ifdef DEBUG
	rtt_println("AK8963 compass connected!\n");
#endif

	return true;
}

/**
 *  @brief      Trigger gyro/accel  self-test for MPU9250
 *  On success/error, the self-test returns a mask representing the sensor(s)
 *  that failed. For each bit, a one (1) represents a "pass" case; conversely,
 *  a zero (0) indicates a failure.
 *
 *  \n The mask is defined as follows:
 *  \n Bit 0:   Accel.
 *  \n Bit 1:   Gyro.
 *
 *  @return     Result mask (see above).
 */
uint8_t mpu9250_accel_gyro_self_test(void)
{
	uint8_t rawData[6] = {0, 0, 0, 0, 0, 0};
	uint8_t selfTest[6];
	int32_t gAvg[3]={0}, aAvg[3]={0}, aSTAvg[3]={0}, gSTAvg[3]={0};
	int32_t factoryTrim[6];
	float aDiff[3], gDiff[3];
	uint8_t FS = 0;
	uint8_t testStatus = 0x00;

	// Write test configuration
	i2cdev_writeByte(mpu9250.dev_addr, MPU9250_SMPLRT_DIV, 0x00); // Set gyro sample rate to 1 kHz
	i2cdev_writeByte(mpu9250.dev_addr, MPU9250_CONFIG, 0x02); // Set gyro sample rate to 1 kHz and DLPF to 92 Hz
	i2cdev_writeByte(mpu9250.dev_addr, MPU9250_GYRO_CONFIG, 1<<FS); // Set full scale range for the gyro to 250 dps
	i2cdev_writeByte(mpu9250.dev_addr, MPU9250_ACCEL_CONFIG_2, 0x02); // Set accelerometer rate to 1 kHz and bandwidth to 92 Hz
	i2cdev_writeByte(mpu9250.dev_addr, MPU9250_ACCEL_CONFIG, 1<<FS); // Set full scale range for the accelerometer to 2 g

	for(uint16_t i = 0; i < 200; i++)
	{
	    // get average current values of gyro and acclerometer
	    i2cdev_readBytes(mpu9250.dev_addr, MPU9250_ACCEL_XOUT_H, 6, &rawData[0]); // Read the six raw data registers into data array
	    aAvg[0] += (int16_t)(((int16_t)rawData[0] << 8) | rawData[1]) ; // Turn the MSB and LSB into a signed 16-bit value
	    aAvg[1] += (int16_t)(((int16_t)rawData[2] << 8) | rawData[3]) ;
	    aAvg[2] += (int16_t)(((int16_t)rawData[4] << 8) | rawData[5]) ;

	    i2cdev_readBytes(mpu9250.dev_addr, MPU9250_GYRO_XOUT_H, 6, rawData); // Read the six raw data registers sequentially into data array
	    gAvg[0] += (int16_t)((int16_t)rawData[0] << 8) | rawData[1]; // Turn the MSB and LSB into a signed 16-bit value
	    gAvg[1] += (int16_t)((int16_t)rawData[2] << 8) | rawData[3];
	    gAvg[2] += (int16_t)((int16_t)rawData[4] << 8) | rawData[5];
	}


	for (uint16_t i = 0; i < 3; i++)
	{ // Get average of 200 values and store as average current readings
		aAvg[i] /= 200;
	    gAvg[i] /= 200;
	}

	  // Configure the accelerometer for self-test
	i2cdev_writeByte(mpu9250.dev_addr, MPU9250_ACCEL_CONFIG, 0xE0); // Enable self test on all three axes and set accelerometer range to +/- 2 g
	i2cdev_writeByte(mpu9250.dev_addr, MPU9250_GYRO_CONFIG, 0xE0); // Enable self test on all three axes and set gyro range to +/- 250 degrees/s
	nrf_delay_ms(50); // Delay a while to let the device stabilize


	for(uint16_t i = 0; i < 200; i++)
	{  // get average self-test values of gyro and acclerometer
		i2cdev_readBytes(mpu9250.dev_addr, MPU9250_ACCEL_XOUT_H, 6, &rawData[0]);  // Read the six raw data registers into data array
		aSTAvg[0] += (int16_t)(((int16_t)rawData[0] << 8) | rawData[1]) ;  // Turn the MSB and LSB into a signed 16-bit value
		aSTAvg[1] += (int16_t)(((int16_t)rawData[2] << 8) | rawData[3]) ;
		aSTAvg[2] += (int16_t)(((int16_t)rawData[4] << 8) | rawData[5]) ;

		i2cdev_readBytes(mpu9250.dev_addr, MPU9250_GYRO_XOUT_H, 6, &rawData[0]);  // Read the six raw data registers sequentially into data array
		gSTAvg[0] += (int16_t)(((int16_t)rawData[0] << 8) | rawData[1]) ;  // Turn the MSB and LSB into a signed 16-bit value
		gSTAvg[1] += (int16_t)(((int16_t)rawData[2] << 8) | rawData[3]) ;
		gSTAvg[2] += (int16_t)(((int16_t)rawData[4] << 8) | rawData[5]) ;
	}

	for (uint16_t i = 0; i < 3; i++)
	{  // Get average of 200 values and store as average self-test readings
		aSTAvg[i] /= 200;
		gSTAvg[i] /= 200;
	}

	// Configure the gyro and accelerometer for normal operation
	i2cdev_writeByte(mpu9250.dev_addr, MPU9250_ACCEL_CONFIG, 0x00);
	i2cdev_writeByte(mpu9250.dev_addr, MPU9250_GYRO_CONFIG, 0x00);
	nrf_delay_ms(25); // Delay a while to let the device stabilize

	// Retrieve accelerometer and gyro factory Self-Test Code from USR_Reg
	//i2cdev_readBytes(mpu9250.dev_addr, MPU9250_SELF_TEST_X_ACCEL, 6, selfTest);
	i2cdev_readByte(mpu9250.dev_addr, MPU9250_SELF_TEST_X_ACCEL, &selfTest[0]); // X-axis accel self-test results
	i2cdev_readByte(mpu9250.dev_addr, MPU9250_SELF_TEST_Y_ACCEL, &selfTest[1]); // Y-axis accel self-test results
	i2cdev_readByte(mpu9250.dev_addr, MPU9250_SELF_TEST_Z_ACCEL, &selfTest[2]); // Z-axis accel self-test results
	i2cdev_readByte(mpu9250.dev_addr, MPU9250_SELF_TEST_X_GYRO, &selfTest[3]);  // X-axis gyro self-test results
	i2cdev_readByte(mpu9250.dev_addr, MPU9250_SELF_TEST_Y_GYRO, &selfTest[4]);  // Y-axis gyro self-test results
	i2cdev_readByte(mpu9250.dev_addr, MPU9250_SELF_TEST_Z_GYRO, &selfTest[5]);  // Z-axis gyro self-test results

	 // Retrieve factory self-test value from self-test code reads
	factoryTrim[0] = (float)(2620/1<<FS)*(pow( 1.01 , ((float)selfTest[0] - 1.0) )); // FT[Xa] factory trim calculation
	factoryTrim[1] = (float)(2620/1<<FS)*(pow( 1.01 , ((float)selfTest[1] - 1.0) )); // FT[Ya] factory trim calculation
	factoryTrim[2] = (float)(2620/1<<FS)*(pow( 1.01 , ((float)selfTest[2] - 1.0) )); // FT[Za] factory trim calculation
	factoryTrim[3] = (float)(2620/1<<FS)*(pow( 1.01 , ((float)selfTest[3] - 1.0) )); // FT[Xg] factory trim calculation
	factoryTrim[4] = (float)(2620/1<<FS)*(pow( 1.01 , ((float)selfTest[4] - 1.0) )); // FT[Yg] factory trim calculation
	factoryTrim[5] = (float)(2620/1<<FS)*(pow( 1.01 , ((float)selfTest[5] - 1.0) )); // FT[Zg] factory trim calculation

	// Report results as a ratio of (STR - FT)/FT; the change from Factory Trim of the Self-Test Response
	// To get percent, must multiply by 100
	for (uint16_t i = 0; i < 3; i++)
	{
		aDiff[i] = 100.0 * ((float)(aSTAvg[i] - aAvg[i])) / factoryTrim[i] - 100.; // Report percent differences
		gDiff[i] = 100.0*((float)(gSTAvg[i] - gAvg[i]))/factoryTrim[i+3] - 100.; // Report percent differences
	}

	if(mpu9250_evaluate_self_test(MPU9250_ST_ACCEL_LOW, MPU9250_ST_ACCEL_HIGH, aDiff[0], "acc X") &&
			mpu9250_evaluate_self_test(MPU9250_ST_ACCEL_LOW, MPU9250_ST_ACCEL_HIGH, aDiff[1], "acc Y") &&
			mpu9250_evaluate_self_test(MPU9250_ST_ACCEL_LOW, MPU9250_ST_ACCEL_HIGH, aDiff[2], "acc Z"))
	{
		testStatus |= 0x01;
	}

	// Check result
	if (mpu9250_evaluate_self_test(MPU9250_ST_GYRO_LOW, MPU9250_ST_GYRO_HIGH, gDiff[0], "gyro X") &&
			mpu9250_evaluate_self_test(MPU9250_ST_GYRO_LOW, MPU9250_ST_GYRO_HIGH, gDiff[1], "gyro Y") &&
			mpu9250_evaluate_self_test(MPU9250_ST_GYRO_LOW, MPU9250_ST_GYRO_HIGH, gDiff[2], "gyro Z"))
	{
	    	testStatus |= 0x02;
	}

#ifdef DEBUG
	if(testStatus)
		rtt_println("Selft test passed\n");
#endif

	return testStatus;
}

/** Get Device ID.
 * This register is used to verify the identity of the device (0b110100).
 * @return Device ID (should be 0x68, 104 dec, 150 oct)
 */
uint8_t mpu9250_get_whoami(void)
{
	uint8_t data;
	i2cdev_readByte(mpu9250.dev_addr, MPU9250_WHO_AM_I, &data);
	return data;
}


/** Get Magnetometer Device ID.
 * This register is used to verify the identity of the device (0b110100).
 * @return Device ID (should be 0x48, 104 dec)
 */
uint8_t mpu9250_get_mag_whoami(void)
{
	uint8_t data;
	i2cdev_readByte(AK8963_ADDRESS, WHO_AM_I_AK8963, &data);
	return data;
}

/** Evaluate the values from a MPU6500 self test.
 * @param low The low limit of the self test
 * @param high The high limit of the self test
 * @param value The value to compare with.
 * @param string A pointer to a string describing the value.
 * @return True if self test within low - high limit, false otherwise
 */
static bool mpu9250_evaluate_self_test(float low, float high, float value, char* string)
{
  if (value < low || value > high)
  {
#ifdef DEBUG
	  char message[40];
	  sprintf(message, "Self test %s [FAIL].\n", string);
	  rtt_println(message);
#endif
	  return false;
  }
  return true;
}

static void mpu9250_set_accel_res()
{
	switch(mpu9250.config_data->accel_fsr)
	{
		case MPU9250_ACCEL_FS_2:
			mpu9250.device_data.accel_res  = 2.0/32768.0;
			break;
		case MPU9250_ACCEL_FS_4:
			mpu9250.device_data.accel_res = 4.0/32768.0;
			break;
		case MPU9250_ACCEL_FS_8:
			mpu9250.device_data.accel_res = 8.0/32768.0;
			break;
		case MPU9250_ACCEL_FS_16:
			mpu9250.device_data.accel_res = 16.0/32768.0;
			break;
	}
}

static void mpu9250_set_gyro_res()
{
	switch(mpu9250.config_data->gyro_fsr)
	{
		case MPU9250_GYRO_FS_250:
			mpu9250.device_data.gyro_res = 250.0/32768.0;
			break;
		case MPU9250_GYRO_FS_500:
			mpu9250.device_data.gyro_res = 500.0/32768.0;
			break;
		case MPU9250_GYRO_FS_1000:
			mpu9250.device_data.gyro_res = 1000.0/32768.0;
			break;
		case MPU9250_GYRO_FS_2000:
			mpu9250.device_data.gyro_res = 2000.0/32768.0;
			break;
	}
}

static void mpu9250_set_mag_res()
{
	switch(mpu9250.config_data->mag_fsr)
	{
 	// Possible magnetometer scales (and their register bit settings) are:
	// 14 bit resolution (0) and 16 bit resolution (1)
		case MPU9250_MAG_FS_14:
			mpu9250.device_data.mag_res = 10.*4912./8190.; // Proper scale to return Tesla
			break;
		case MPU9250_MAG_FS_16:
			mpu9250.device_data.mag_res = 4900.0f * 10.0f / 32768.0f;//Proper scale to return Tesla
			break;
	}
}

/**
 *  @brief      Get bias calibration for accel and gyro of MPU9250. These values
 * are needed to calculate the output in g's or dgs
 *
 *  @return    none
 */
void mpu9250_accel_gyro_bias_cal()
{
	int32_t accel_bias[3] = {0, 0, 0}, gyro_bias[3] = {0, 0, 0};

	mpu9250_config_cal();
	mpu9250_calculate_bias(accel_bias, gyro_bias);

	mpu9250_set_gyro_bias_reg(gyro_bias);
	mpu9250_set_accel_bias_reg(accel_bias);
}

/**
 *  @brief   Set mpu9250 configuration to undergone bias calibration
 *
 *  @return    none
 */
static void mpu9250_config_cal()
{
	//Reset device
	i2cdev_writeByte(mpu9250.dev_addr, MPU9250_PWR_MGMT_1, 0x80);
	nrf_delay_ms(100);

	// get stable time source; Auto select clock source to be PLL gyroscope reference if ready
	// else use the internal oscillator, bits 2:0 = 001
	i2cdev_writeByte(mpu9250.dev_addr, MPU9250_PWR_MGMT_1, 0x01);
	i2cdev_writeByte(mpu9250.dev_addr, MPU9250_PWR_MGMT_2, 0x00);
	nrf_delay_ms(200);


	// Configure device for bias calculation
	i2cdev_writeByte(mpu9250.dev_addr, MPU9250_INT_ENABLE, 0x00);   // Disable all interrupts
	i2cdev_writeByte(mpu9250.dev_addr, MPU9250_FIFO_EN, 0x00);      // Disable FIFO
	i2cdev_writeByte(mpu9250.dev_addr, MPU9250_PWR_MGMT_1, 0x00);   // Turn on internal clock source
	i2cdev_writeByte(mpu9250.dev_addr, MPU9250_I2C_MST_CTRL, 0x00); // Disable I2C master
	i2cdev_writeByte(mpu9250.dev_addr, MPU9250_USER_CTRL, 0x00);    // Disable FIFO and I2C master modes
	i2cdev_writeByte(mpu9250.dev_addr, MPU9250_USER_CTRL, 0x0C);    // Reset FIFO and DMP
	nrf_delay_ms(15);

	// Configure MPU6050 gyro and accelerometer for bias calculation
	i2cdev_writeByte(mpu9250.dev_addr, MPU9250_CONFIG, 0x01);      // Set low-pass filter to 188 Hz
	i2cdev_writeByte(mpu9250.dev_addr, MPU9250_SMPLRT_DIV, 0x00);  // Set sample rate to 1 kHz
	i2cdev_writeByte(mpu9250.dev_addr, MPU9250_GYRO_CONFIG, 0x00);  // Set gyro full-scale to 250 degrees per second, maximum sensitivity
	i2cdev_writeByte(mpu9250.dev_addr, MPU9250_ACCEL_CONFIG, 0x00); // Set accelerometer full-scale to +-2 g, maximun sensitivity
}

/**
 *  @brief   Calculates accel bias and gyro bias
 *
 *  @return    none
 */
static void mpu9250_calculate_bias(int32_t *accel_bias, int32_t *gyro_bias)
{
	uint8_t data[12] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0 ,0, 0};
	uint16_t packet_count, fifo_count;
	unsigned long accel_sensitivity = 32768/2;  // = 16384 LSB/g

	// Configure FIFO to capture accelerometer and gyro data for bias calculation
	i2cdev_writeByte(mpu9250.dev_addr, MPU9250_USER_CTRL, 0x40);   // Enable FIFO
	i2cdev_writeByte(mpu9250.dev_addr, MPU9250_FIFO_EN, 0x78);     // Enable gyro and accelerometer sensors for FIFO  (max size 512 bytes in MPU-9150)
	nrf_delay_ms(40); // accumulate 40 samples in 40 milliseconds = 480 bytes

	// At end of sample accumulation, turn off FIFO sensor read
	i2cdev_writeByte(mpu9250.dev_addr, MPU9250_FIFO_EN, 0x00);        // Disable gyro and accelerometer sensors for FIFO
	i2cdev_readBytes(mpu9250.dev_addr, MPU9250_FIFO_COUNTH, 2, &data[0]); // read FIFO sample count
	fifo_count = ((uint16_t)data[0] << 8) | data[1];
	packet_count = fifo_count/12;// How many sets of full gyro and accelerometer data for averaging

	for (uint16_t i = 0; i < packet_count; i++)
	{
		int16_t accel_temp[3] = {0, 0, 0}, gyro_temp[3] = {0, 0, 0};
		i2cdev_readBytes(mpu9250.dev_addr, MPU9250_FIFO_R_W, 12, &data[0]); // read data for averaging
	    accel_temp[0] = (int16_t) (((int16_t)data[0] << 8) | data[1]  ) ;  // Form signed 16-bit integer for each sample in FIFO
	    accel_temp[1] = (int16_t) (((int16_t)data[2] << 8) | data[3]  ) ;
	    accel_temp[2] = (int16_t) (((int16_t)data[4] << 8) | data[5]  ) ;
	    gyro_temp[0]  = (int16_t) (((int16_t)data[6] << 8) | data[7]  ) ;
	    gyro_temp[1]  = (int16_t) (((int16_t)data[8] << 8) | data[9]  ) ;
	    gyro_temp[2]  = (int16_t) (((int16_t)data[10] << 8) | data[11]) ;

	    accel_bias[0] += (int32_t) accel_temp[0]; // Sum individual signed 16-bit biases to get accumulated signed 32-bit biases
	    accel_bias[1] += (int32_t) accel_temp[1];
	    accel_bias[2] += (int32_t) accel_temp[2];
	    gyro_bias[0]  += (int32_t) gyro_temp[0];
	    gyro_bias[1]  += (int32_t) gyro_temp[1];
	    gyro_bias[2]  += (int32_t) gyro_temp[2];

	}

	accel_bias[0] /= (int32_t) packet_count; // Normalize sums to get average count biases
	accel_bias[1] /= (int32_t) packet_count;
	accel_bias[2] /= (int32_t) packet_count;
	gyro_bias[0]  /= (int32_t) packet_count;
	gyro_bias[1]  /= (int32_t) packet_count;
    gyro_bias[2]  /= (int32_t) packet_count;

    if(accel_bias[2] > 0L) {accel_bias[2] -= (int32_t) accel_sensitivity;}  // Remove gravity from the z-axis accelerometer bias calculation
    else {accel_bias[2] += (int32_t) accel_sensitivity;}
}

// Construct the gyro biases for push to the hardware gyro bias registers,
//which are reset to zero upon device startup
static void mpu9250_set_gyro_bias_reg(int32_t *gyro_bias)
{
	float gyro_sensitivity = 32768/250;
	uint8_t data[3] = {0, 0, 0};

	data[0] = (-gyro_bias[0]/4  >> 8) & 0xFF; // Divide by 4 to get 32.9 LSB per deg/s to conform to expected bias input format
	data[1] = (-gyro_bias[0]/4)       & 0xFF; // Biases are additive, so change sign on calculated average gyro biases
	data[2] = (-gyro_bias[1]/4  >> 8) & 0xFF;
	data[3] = (-gyro_bias[1]/4)       & 0xFF;
	data[4] = (-gyro_bias[2]/4  >> 8) & 0xFF;
	data[5] = (-gyro_bias[2]/4)       & 0xFF;

	// Push gyro biases to hardware registers
	i2cdev_writeByte(mpu9250.dev_addr, MPU9250_XG_OFFSET_H, data[0]);
	i2cdev_writeByte(mpu9250.dev_addr, MPU9250_XG_OFFSET_L, data[1]);
	i2cdev_writeByte(mpu9250.dev_addr, MPU9250_YG_OFFSET_H, data[2]);
	i2cdev_writeByte(mpu9250.dev_addr, MPU9250_YG_OFFSET_L, data[3]);
	i2cdev_writeByte(mpu9250.dev_addr, MPU9250_ZG_OFFSET_H, data[4]);
	i2cdev_writeByte(mpu9250.dev_addr, MPU9250_ZG_OFFSET_L, data[5]);

    mpu9250.device_data.gyro_bias.x = (float)gyro_bias[0]/gyro_sensitivity;
    mpu9250.device_data.gyro_bias.y = (float)gyro_bias[1]/gyro_sensitivity;
    mpu9250.device_data.gyro_bias.z = (float)gyro_bias[2]/gyro_sensitivity;

#ifdef DEBUG
    rtt_print_float("Gyro bias x:", mpu9250.device_data.gyro_bias.x);
    rtt_print_float("Gyro bias y:", mpu9250.device_data.gyro_bias.y);
    rtt_print_float("Gyro bias z:", mpu9250.device_data.gyro_bias.z);
#endif
}

/**
* @brief Read biases to the accel bias 6500 registers.
* This function reads from the MPU6500 accel offset cancellations registers.
* Push biases to the accel bias 6500 registers.
* This function expects biases relative to the current sensor output, and
* these biases will be added to the factory-supplied values.
* The format are G in +-2G format. The register is initialized with OTP
* factory trim values.
* @param[in] accel_bias returned structure with the accel bias
* @return none
*/
static void mpu9250_set_accel_bias_reg(int32_t *accel_bias)
{
	uint8_t data[6] = {0,0,0,0,0,0};
	int32_t accel_bias_reg[3] = {0, 0, 0}; // A place to hold the factory accelerometer trim biases
	int32_t mask = 0x0001;
	uint8_t mask_bit[3] = {0, 0, 0};
	unsigned long accel_sensitivity = 32768/2;  // = 16384 LSB/g

	i2cdev_readBytes(mpu9250.dev_addr, MPU9250_XA_OFFSET_H, 2, &data[0]); // Read factory accelerometer trim values
	accel_bias_reg[0] = (int32_t) (((int16_t)data[0] << 8) | data[1]);

	i2cdev_readBytes(mpu9250.dev_addr, MPU9250_YA_OFFSET_H, 2, &data[0]);
	accel_bias_reg[1] = (int32_t) (((int16_t)data[0] << 8) | data[1]);

	i2cdev_readBytes(mpu9250.dev_addr, MPU9250_ZA_OFFSET_H, 2, &data[0]);
	accel_bias_reg[2] = (int32_t) (((int16_t)data[0] << 8) | data[1]);

	 //bit 0 of the 2 byte bias is for temp comp
	 //calculations need to compensate for this
	 for(uint16_t i=0; i<3; i++)
	 {
		 if(accel_bias_reg[i]&mask)
		 {
			 mask_bit[i] = 0x01;
		 }
	 }

	 for(uint16_t i = 0; i < 3; i++)
	 {
		 accel_bias[i] /= 4; //+-2G format ---> +-8G format
	 }

	 accel_bias_reg[0] -= accel_bias[0];
	 accel_bias_reg[1] -= accel_bias[1];
	 accel_bias_reg[2] -= accel_bias[2];

	 data[0] = (accel_bias_reg[0] >> 8) & 0xff;
	 data[1] = (accel_bias_reg[0]) & 0xff;
	 data[1] = data[1]|mask_bit[0];
	 data[2] = (accel_bias_reg[1] >> 8) & 0xff;
	 data[3] = (accel_bias_reg[1]) & 0xff;
	 data[3] = data[3]|mask_bit[1];
	 data[4] = (accel_bias_reg[2] >> 8) & 0xff;
	 data[5] = (accel_bias_reg[2]) & 0xff;
	 data[5] = data[5]|mask_bit[2];

	 // Apparently this is not working for the acceleration biases in the MPU-9250
	 // Are we handling the temperature correction bit properly?
	 // Push accelerometer biases to hardware registers
	 i2cdev_writeByte(mpu9250.dev_addr, MPU9250_XA_OFFSET_H, data[0]);
	 i2cdev_writeByte(mpu9250.dev_addr, MPU9250_XA_OFFSET_L, data[1]);
	 i2cdev_writeByte(mpu9250.dev_addr, MPU9250_YA_OFFSET_H, data[2]);
	 i2cdev_writeByte(mpu9250.dev_addr, MPU9250_YA_OFFSET_L, data[3]);
	 i2cdev_writeByte(mpu9250.dev_addr, MPU9250_ZA_OFFSET_H, data[4]);
	 i2cdev_writeByte(mpu9250.dev_addr, MPU9250_ZA_OFFSET_L, data[5]);

	 mpu9250.device_data.accel_bias.x = (float)accel_bias[0]/(float)accel_sensitivity;
	 mpu9250.device_data.accel_bias.y = (float)accel_bias[1]/(float)accel_sensitivity;
	 mpu9250.device_data.accel_bias.z = (float)accel_bias[2]/(float)accel_sensitivity;

#ifdef DEBUG
    rtt_print_float("Accel bias x:", mpu9250.device_data.accel_bias.x);
    rtt_print_float("Accel bias y:", mpu9250.device_data.accel_bias.y);
    rtt_print_float("Accel bias z:", mpu9250.device_data.accel_bias.z);
#endif
}

static void mpu9250_get_mag_cal_data()
{
	// First extract the factory calibration for each magnetometer axis
	uint8_t rawData[3] = {0, 0, 0};  // x/y/z gyro calibration data stored here
	i2cdev_writeByte(mpu9250.mag_addr, AK8963_CNTL, 0x00); // Power down magnetometer
	nrf_delay_ms(10);

	i2cdev_writeByte(mpu9250.mag_addr, AK8963_CNTL, 0x0F); // Enter Fuse ROM access mode
	nrf_delay_ms(10);

	bool result = i2cdev_readBytes(mpu9250.mag_addr, AK8963_ASAX, 3, &rawData[0]);  // Read the x-, y-, and z-axis calibration values
	mpu9250.device_data.mag_calibration.x =  (float)(rawData[0] - 128)/256. + 1.;   // Return x-axis sensitivity adjustment values, etc.
	mpu9250.device_data.mag_calibration.y =  (float)(rawData[1] - 128)/256. + 1.;
	mpu9250.device_data.mag_calibration.z =  (float)(rawData[2] - 128)/256. + 1.;

	result = i2cdev_writeByte(mpu9250.mag_addr, AK8963_CNTL, 0x00); // Power down magnetometer
	nrf_delay_ms(10);

	// Configure the magnetometer for continuous read and highest resolution
	// set Mscale bit 4 to 1 (0) to enable 16 (14) bit resolution in CNTL register,
	// and enable continuous mode data acquisition Mmode (bits [3:0]), 0010 for 8 Hz and 0110 for 100 Hz sample rates
	result = i2cdev_writeByte(mpu9250.mag_addr, AK8963_CNTL, 0x01 << 4 | 0x06); // Set magnetometer data resolution and sample ODR
	if(result)
		nrf_delay_ms(10);

#ifdef DEBUG
    rtt_print_float("Mag cal x:", mpu9250.device_data.mag_calibration.x);
    rtt_print_float("Mag cal y:", mpu9250.device_data.mag_calibration.y);
    rtt_print_float("Mag cal z:", mpu9250.device_data.mag_calibration.z);
#endif
}

#ifdef MAG_CALIBRATION
static void mpu9250_get_mag_bias()
{
	uint16_t sample_count = 0;
	int32_t mag_bias[3] = {0, 0, 0}, mag_scale[3] = {0, 0, 0};
	int16_t mag_max[3] = {-32767, -32767, -32767}, mag_min[3] = {32767, 32767, 32767}, mag_temp[3] = {0, 0, 0};

	rtt_println("Mag Calibration is about to start\n");
	nrf_delay_ms(4000);
	rtt_println("Mag Calibration: Started\n");
	//rtt_println("Put Mag in X+ position");
	sample_count = 2000;
	for(uint16_t i = 0; i < sample_count; i++) {
//		if(i == 1000) rtt_println("Put Mag in X- position\n");
//		if(i == 2000) rtt_println("Put Mag in Y+ position\n");
//		if(i == 3000) rtt_println("Put Mag in Y- position\n");
//		if(i == 4000) rtt_println("Put Mag in Z+ position\n");
//		if(i == 5000) rtt_println("Put Mag in Z- position\n");
		bool res = mpu9250_read_mag_data(mag_temp);  // Read the mag data

		if(res && !(mag_temp[0] == 0 && mag_temp[1] == 0 && mag_temp[2] == 0))
		{
			for (int j = 0; j < 3; j++)
			{
			  if(mag_temp[j] > mag_max[j]) mag_max[j] = mag_temp[j];
			  if(mag_temp[j] < mag_min[j]) mag_min[j] = mag_temp[j];
			}
			nrf_delay_ms(12);  // at 100 Hz ODR, new mag data is available every 10 ms
		}
		else{
			rtt_println("Overflow!\n");
			rtt_print_float("Bad Reading:", mag_temp[0]+ mag_temp[1]+ mag_temp[2]);
		}
	}

	mag_bias[0]  = (mag_max[0] + mag_min[0])/2;  // get average x mag bias in counts
	mag_bias[1]  = (mag_max[1] + mag_min[1])/2;  // get average y mag bias in counts
	mag_bias[2]  = (mag_max[2] + mag_min[2])/2;  // get average z mag bias in counts

	mpu9250.device_data.mag_bias.x = (float) mag_bias[0]*mpu9250.device_data.mag_res*mpu9250.device_data.mag_calibration.x;  // save mag biases in G for main program
	mpu9250.device_data.mag_bias.y = (float) mag_bias[1]*mpu9250.device_data.mag_res*mpu9250.device_data.mag_calibration.y;
	mpu9250.device_data.mag_bias.z = (float) mag_bias[2]*mpu9250.device_data.mag_res*mpu9250.device_data.mag_calibration.z;

	// Get soft iron correction estimate
	mag_scale[0]  = (mag_max[0] - mag_min[0])/2;  // get average x axis max chord length in counts
	mag_scale[1]  = (mag_max[1] - mag_min[1])/2;  // get average y axis max chord length in counts
	mag_scale[2]  = (mag_max[2] - mag_min[2])/2;  // get average z axis max chord length in counts

	float avg_rad = mag_scale[0] + mag_scale[1] + mag_scale[2];
	avg_rad /= 3.0;

	mpu9250.device_data.mag_scale.x = avg_rad/((float)mag_scale[0]);
	mpu9250.device_data.mag_scale.y = avg_rad/((float)mag_scale[1]);
	mpu9250.device_data.mag_scale.z = avg_rad/((float)mag_scale[2]);


	rtt_println("Mag Calibration done!");

#ifdef DEBUG
    rtt_print_float("Mag bias x:", mpu9250.device_data.mag_bias.x);
    rtt_print_float("Mag bias y:", mpu9250.device_data.mag_bias.y);
    rtt_print_float("Mag bias z:", mpu9250.device_data.mag_bias.z);
    rtt_print_float("Mag scale x:", mpu9250.device_data.mag_scale.x);
    rtt_print_float("Mag scale y:", mpu9250.device_data.mag_scale.y);
    rtt_print_float("Mag scale z:", mpu9250.device_data.mag_scale.z);
    rtt_print_float("Max z:", mag_max[3]);
#endif
}
#endif

/**
* @brief Set MPU9250 configuration indicated by device_config
* @param[in] device_config: configuration to be set
* @return none
*/
void mpu9250_set_config()
{
	i2cdev_writeByte(mpu9250.dev_addr, MPU9250_PWR_MGMT_1, 0x80); // Clear sleep mode bit (6), enable all sensors
	nrf_delay_ms(100);// Wait for all registers to reset

	// Configure Gyro and Thermometer
	// Disable FSYNC and set thermometer and gyro bandwidth
	i2cdev_writeByte(mpu9250.dev_addr, MPU9250_CONFIG, 0x00 | mpu9250.config_data->gyro_temp_dlpf);

	// Set sample rate = gyroscope output rate/(1 + SMPLRT_DIV)
	i2cdev_writeByte(mpu9250.dev_addr, MPU9250_SMPLRT_DIV, mpu9250.config_data->sample_rate_div);

	uint8_t config;
	// Set gyroscope full scale range
	// Range selects FS_SEL and GFS_SEL are 0 - 3, so 2-bit values are left-shifted into positions 4:3
	i2cdev_readByte(mpu9250.dev_addr, MPU9250_GYRO_CONFIG, &config);
	i2cdev_writeByte(mpu9250.dev_addr, MPU9250_GYRO_CONFIG, config & ~mpu9250.config_data->gyro_fchoice); // Set Fchoice bits [1:0]
	i2cdev_writeByte(mpu9250.dev_addr, MPU9250_GYRO_CONFIG, config & ~0x18); // Clear GFS bits [4:3]
	i2cdev_writeByte(mpu9250.dev_addr, MPU9250_GYRO_CONFIG, config | mpu9250.config_data->gyro_fsr << 3); // Set full scale range for the gyro

	// Set accelerometer full-scale range configuration
	i2cdev_readByte(mpu9250.dev_addr, MPU9250_ACCEL_CONFIG, &config);
	//  writeRegister(ACCEL_CONFIG, c & ~0xE0); // Clear self-test bits [7:5]
	i2cdev_writeByte(mpu9250.dev_addr, MPU9250_ACCEL_CONFIG, config & ~0x18); // Clear AFS bits [4:3]
	i2cdev_writeByte(mpu9250.dev_addr, MPU9250_ACCEL_CONFIG, config | mpu9250.config_data->accel_fsr << 3); // Set full scale range for the accelerometer

	// Set accelerometer sample rate configuration
	// It is possible to get a 4 kHz sample rate from the accelerometer by choosing 1 for
	// accel_fchoice_b bit [3]; in this case the bandwidth is 1.13 kHz
	i2cdev_readByte(mpu9250.dev_addr, MPU9250_ACCEL_CONFIG_2, &config);
	i2cdev_writeByte(mpu9250.dev_addr, MPU9250_ACCEL_CONFIG_2, config & ~0x0F); // clear accel_fchoice_b (bit 3) and A_DLPFG (bits [2:0])
	uint8_t accel_fchoice_b = ~mpu9250.config_data->accel_fchoice & 0x01;
	i2cdev_writeByte(mpu9250.dev_addr, MPU9250_ACCEL_CONFIG_2,
						config | accel_fchoice_b << 3 | mpu9250.config_data->accel_dlpf); // set accel_fchoice_b and A_DLPFG

	if(mpu9250.config_data->fifo_enable == 1)
	{
		// Configure FIFO to capture accelerometer and gyro data for bias calculation
		i2cdev_writeByte(mpu9250.dev_addr, MPU9250_USER_CTRL, 0x40);   // Enable FIFO
		i2cdev_writeByte(mpu9250.dev_addr, MPU9250_FIFO_EN, 0x78);     // Enable gyro and accelerometer sensors for FIFO  (max size 512 bytes in MPU-9150)
		nrf_delay_ms(40);
	}

	// Configure Interrupts and Bypass Enable
	// Set interrupt pin active high, push-pull, hold interrupt pin level HIGH until interrupt cleared,
	// clear on read of INT_STATUS, and enable I2C_BYPASS_EN so additional chips
	// can join the I2C bus and all can be controlled by the Arduino as master
	i2cdev_writeByte(mpu9250.dev_addr, MPU9250_INT_PIN_CFG, 0x22);
	i2cdev_writeByte(mpu9250.dev_addr, MPU9250_INT_ENABLE, 0x00 | mpu9250.config_data->int_enable);  // Enable data ready (bit 0) interrupt
	nrf_delay_ms(100);
}

void mpu9250_set_mag_config()
{
	// Configure the magnetometer for continuous read and indicated resolution
	// set Mscale bit 4 to 1 (0) to enable 16 (14) bit resolution in CNTL register,
	// and enable continuous mode data acquisition Mmode (bits [3:0]), 0010 for 8 Hz and 0110 for 100 Hz sample rates
	i2cdev_writeByte(AK8963_ADDRESS, AK8963_CNTL,
			mpu9250.config_data->mag_fsr << 4 |  mpu9250.config_data->mag_operating_mode); // Set magnetometer data resolution and sample ODR
	nrf_delay_ms(10);
}

/**
* @brief Read ACCEL_#OUT Registers to get accel raw data
* @param[out] data accel raw data returned
* @return if success true, otherwise false
*/
bool mpu9250_read_accel_data(int16_t *data){
	uint8_t raw_data[6] = {0, 0, 0, 0, 0, 0};

	if(!i2cdev_readBytes(mpu9250.dev_addr, MPU9250_ACCEL_XOUT_H, 6, raw_data)){
		return false;
	}

	data[0] = ((int16_t)raw_data[0] << 8) | raw_data[1] ;  // Turn the MSB and LSB into a signed 16-bit value
	data[1] = ((int16_t)raw_data[2] << 8) | raw_data[3] ;
	data[2] = ((int16_t)raw_data[4] << 8) | raw_data[5] ;

	return true;
}

/**
* @brief Get accel data in Gs
* @param[out] data accel data in Gs returned
* @return if success true, otherwise false
*/
bool mpu9250_get_accel_data_gs(axis_float_t *data){
	int16_t raw_data[3] = {0, 0, 0};

	if(!mpu9250_read_accel_data(raw_data))
		return false;

	data->x = (float)raw_data[0]*mpu9250.device_data.accel_res - mpu9250.device_data.accel_bias.x;
	data->y = (float)raw_data[1]*mpu9250.device_data.accel_res - mpu9250.device_data.accel_bias.y;
	data->z = (float)raw_data[2]*mpu9250.device_data.accel_res - mpu9250.device_data.accel_bias.z;

	return true;
}

/**
* @brief Read GYRO_#OUT Registers to get gyro raw data
* @param[out] data: accel gyro data returned
* @return if success true, otherwise false
*/
bool mpu9250_read_gyro_data(int16_t *data){
	uint8_t raw_data[6] = {0, 0, 0, 0, 0, 0};

	if(!i2cdev_readBytes(mpu9250.dev_addr, MPU9250_GYRO_XOUT_H, 6, raw_data)){
		return false;
	}

	data[0] = ((int16_t)raw_data[0] << 8) | raw_data[1] ;  // Turn the MSB and LSB into a signed 16-bit value
	data[1] = ((int16_t)raw_data[2] << 8) | raw_data[3] ;
	data[2] = ((int16_t)raw_data[4] << 8) | raw_data[5] ;

	return true;
}

/**
* @brief Get gyro data in radians per second (rds)
* @param[out] data: gyro data in rps returned
* @return if success true, otherwise false
*/
bool mpu9250_get_gyro_data_rps(axis_float_t *data)
{
	bool result = mpu9250_get_gyro_data_dps(data);

	data->x = data->x *PI/180.0f;
	data->y = data->y *PI/180.0f;
	data->z = data->z *PI/180.0f;

	return result;
}

/**
* @brief Get gyro data in degrees per second (dps)
* @param[out] data: gyro data in dps returned
* @return if success true, otherwise false
*/
bool mpu9250_get_gyro_data_dps(axis_float_t *data){
	int16_t raw_data[3] = {0, 0, 0};

	if(!mpu9250_read_gyro_data(raw_data))
		return false;

	data->x = (float)raw_data[0]*mpu9250.device_data.gyro_res - mpu9250.device_data.gyro_bias.x;
	data->y = (float)raw_data[1]*mpu9250.device_data.gyro_res - mpu9250.device_data.gyro_bias.y;
	data->z = (float)raw_data[2]*mpu9250.device_data.gyro_res - mpu9250.device_data.gyro_bias.z;

	return true;
}

/**
* @brief Read MAG_#OUT Registers to get mag raw data
* @param[out] data: mag raw data returned
* @return if success true, otherwise false
*/
bool mpu9250_read_mag_data(int16_t *data){
	uint8_t raw_data[7] = {0, 0, 0, 0, 0, 0, 0};
	uint8_t st1_reg_byte = 0;

	if(!i2cdev_readByte(mpu9250.mag_addr, AK8963_ST1, &st1_reg_byte))
		return false;

	if((st1_reg_byte & 0x01) == 0x01){
		if(!i2cdev_readBytes(mpu9250.mag_addr, AK8963_XOUT_L, 7, raw_data))// Read the six raw data and ST2 registers sequentially into data array
				return false;

		uint8_t overflow = raw_data[6];
	    if(!(overflow & 0x08)) { // Check if magnetic sensor overflow set, if not then report data
	    	data[0] = ((int16_t)raw_data[1] << 8) | raw_data[0] ;  // Turn the MSB and LSB into a signed 16-bit value
	    	data[1] = ((int16_t)raw_data[3] << 8) | raw_data[2] ;  // Data stored as little Endian
	    	data[2] = ((int16_t)raw_data[5] << 8) | raw_data[4] ;
	    }
	    else return false;
	}

	return true;
}

/**
* @brief Get mag data in miligauss,  Include factory calibration per data
* sheet and user environmental corrections
* @param[out] data: mag data in miligauss returned
* @return if success true, otherwise false
*/
bool mpu9250_get_mag_data_miligauss(axis_float_t *data){
	int16_t raw_data[3] = {0, 0, 0};

	data->x = 0;
	data->y = 0;
	data->z = 0;

	if(!mpu9250_read_mag_data(raw_data))
		return false;

	if(raw_data[0] == 0 && raw_data[1] == 0 && raw_data[2] == 0 )
	{
		return false;
	}

	float magX = (float)raw_data[0]*mpu9250.device_data.mag_res*mpu9250.device_data.mag_calibration.x / 1000 - mpu9250.device_data.mag_bias.x;
	float magY = (float)raw_data[1]*mpu9250.device_data.mag_res*mpu9250.device_data.mag_calibration.y / 1000 - mpu9250.device_data.mag_bias.y;
	float magZ = (float)raw_data[2]*mpu9250.device_data.mag_res*mpu9250.device_data.mag_calibration.z / 1000 - mpu9250.device_data.mag_bias.z;

//	data->x *= mpu9250.device_data.mag_scale.x;
//	data->y *= mpu9250.device_data.mag_scale.y;
//	data->z *= mpu9250.device_data.mag_scale.z;
	data->x = M11*magX + M12*magY + M13*magZ;
	data->y = M21*magX + M22 * magY + M23 * magZ;
	data->z = M31 * magX + M32 * magY + M33 * magZ;

	return true;
}

void mpu9250_set_mag_cal_data()
{
	mpu9250.device_data.mag_bias.x = MAG_BIAS_X;
	mpu9250.device_data.mag_bias.y = MAG_BIAS_Y;
	mpu9250.device_data.mag_bias.z = MAG_BIAS_Z;
	mpu9250.device_data.mag_scale.x = MAG_SCALE_X;
	mpu9250.device_data.mag_scale.y = MAG_SCALE_Y;
	mpu9250.device_data.mag_scale.z = MAG_SCALE_Z;
}

