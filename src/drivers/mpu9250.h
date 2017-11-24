/*
 * mpu9250.h
 *
 *  Created on: 22 ago. 2017
 *      Author: Diego Aguirre
 *
 *      based on MPU-9250 library by femtoio github user
 *      https://github.com/femtoio/MPU9250/blob/master/MPU9250.h
 */

#ifndef _MPU9250_H_
#define _MPU9250_H_

#include <stdint.h>
#include <stdbool.h>

typedef struct{
	int16_t x;
	int16_t y;
	int16_t z;
}axis16_t;

typedef struct{
	float x;
	float y;
	float z;
}axis_float_t;

typedef struct{
	uint8_t accel_fsr;
	uint8_t gyro_fsr;
	uint8_t mag_fsr;
	uint8_t gyro_fchoice;
	uint8_t accel_fchoice;
	uint8_t gyro_temp_dlpf;
	uint8_t accel_dlpf;
	uint8_t mag_operating_mode;
	uint8_t fifo_enable;
	uint8_t int_enable;
	uint8_t sample_rate_div;
}mpu9250_config_t;

//typedef struct{
//	uint8_t mag_fsr;
//	uint8_t mode;
//	uint8_t sample_rate;
//}mag_config_t;

typedef struct{
	axis_float_t accel_bias;
	axis_float_t gyro_bias;
	axis_float_t mag_bias;
	axis_float_t mag_scale;
	axis_float_t mag_calibration;
	float accel_res;
	float gyro_res;
	float mag_res;
}mpu9250_dev_data_t;

typedef struct{
	const uint8_t dev_addr;
	const uint8_t mag_addr;
	mpu9250_dev_data_t device_data;
	const mpu9250_config_t *config_data;
}mpu9250_t;

// Test limits
#define MPU9250_ST_GYRO_LOW      (-14.0)  // %
#define MPU9250_ST_GYRO_HIGH     14.0  // %
#define MPU9250_ST_ACCEL_LOW     (-14.0)  // %
#define MPU9250_ST_ACCEL_HIGH    14.0  // %

#define MPU9250_GYRO_FS_250         0x00
#define MPU9250_GYRO_FS_500         0x01
#define MPU9250_GYRO_FS_1000        0x02
#define MPU9250_GYRO_FS_2000        0x03

#define MPU9250_ACCEL_FS_2          0x00
#define MPU9250_ACCEL_FS_4          0x01
#define MPU9250_ACCEL_FS_8          0x02
#define MPU9250_ACCEL_FS_16         0x03

#define MPU9250_MAG_MODE_POWER_DOWN			0x00
#define MPU9250_MAG_MODE_SINGLE				0x01
#define MPU9250_MAG_MODE_CONTINUOUS_1		0x02
#define MPU9250_MAG_MODE_CONTINUOUS_2		0x06
#define MPU9250_MAG_MODE_EXT_TRIGGER		0x04
#define MPU9250_MAG_MODE_SELF_TEST			0x08
#define MPU9250_MAG_MODE_ROM				0x0F


#define MPU9250_MAG_FS_14				0x00
#define MPU9250_MAG_FS_16				0x01

#define MPU9250_I2C_CLOCK_SPEED                  400000UL // I2C is 400KHz max
#define MPU9250_I2C_ADDRESS                      0x68 // This address is used by MPU9250 when ADC0 pin is logic low
#define MPU9250_I2C_ADDRESS_ALT                  0x69 // This address is used by MPU9250 when ADC0 pin is logic high

// Note, this is the reset value for all registers except
// - Register 107 (0x01) Power Management 1
// - Register 117 (0x71) WHO_AM_I
#define MPU9250_REG_RESET                        0x00

// From section 7.5 SPI Interface
// SPI read and write operations are completed in 16 or more clock cycles (two or more bytes). The
// first byte contains the SPI A ddress, and the following byte(s) contain(s) the SPI data. The first
// bit of the first byte contains the Read/Write bit and indicates the Read (1) or Write (0) operation.
// The following 7 bits contain the Register Address. In cases of multiple-byte Read/Writes, data is
// two or more bytes...
#define MPU9250_READ_MASK                        0x80

// Self Test, Gyro
#define MPU9250_SELF_TEST_X_GYRO                 0x00
#define MPU9250_SELF_TEST_Y_GYRO                 0x01
#define MPU9250_SELF_TEST_Z_GYRO                 0x02



// Self Test, Accelerometer
#define MPU9250_SELF_TEST_X_ACCEL                0x0d
#define MPU9250_SELF_TEST_Y_ACCEL                0x0e
#define MPU9250_SELF_TEST_Z_ACCEL                0x0f

 // Gyro Offset
#define MPU9250_XG_OFFSET_H                      0x13
#define MPU9250_XG_OFFSET_L                      0x14
#define MPU9250_YG_OFFSET_H                      0x15
#define MPU9250_YG_OFFSET_L                      0x16
#define MPU9250_ZG_OFFSET_H                      0x17
#define MPU9250_ZG_OFFSET_L                      0x18


#define MPU9250_SMPLRT_DIV                       0x19

// Config
#define MPU9250_CONFIG                           0x1a
#define MPU9250_GYRO_CONFIG                      0x1b
#define MPU9250_ACCEL_CONFIG                     0x1c
#define MPU9250_ACCEL_CONFIG_2                   0x1d
#define MPU9250_LP_ACCEL_ODR                     0x1e

#define MPU9250_WOM_THR                          0x1f

#define MPU9250_FIFO_EN                          0x23

// I2C
#define MPU9250_I2C_MST_CTRL                     0x24
#define MPU9250_I2C_SLV0_ADDR                    0x25
#define MPU9250_I2C_SLV0_REG                     0x26
#define MPU9250_I2C_SLV0_CTRL                    0x27

#define MPU9250_I2C_SLV1_ADDR                    0x28
#define MPU9250_I2C_SLV1_REG                     0x29
#define MPU9250_I2C_SLV1_CTRL                    0x2a

#define MPU9250_I2C_SLV2_ADDR                    0x2b
#define MPU9250_I2C_SLV2_REG                     0x2c
#define MPU9250_I2C_SLV2_CTRL                    0x2d

#define MPU9250_I2C_SLV3_ADDR                    0x2e
#define MPU9250_I2C_SLV3_REG                     0x2f
#define MPU9250_I2C_SLV3_CTRL                    0x30

#define MPU9250_I2C_SLV4_ADDR                    0x31
#define MPU9250_I2C_SLV4_REG                     0x32
#define MPU9250_I2C_SLV4_DO                      0x33
#define MPU9250_I2C_SLV4_CTRL                    0x34
#define MPU9250_I2C_SLV4_DI                      0x35

#define MPU9250_I2C_MST_STATUS                   0x36

#define MPU9250_INT_PIN_CFG                      0x37
#define MPU9250_INT_ENABLE                       0x38

#define MPU9250_DMP_INT_STATUS                   0x39 // Check DMP Interrupt, see 0x6d

#define MPU9250_INT_STATUS                       0x3a

// Accel XOUT
#define MPU9250_ACCEL_XOUT_H                     0x3b
#define MPU9250_ACCEL_XOUT_L                     0x3c
#define MPU9250_ACCEL_YOUT_H                     0x3d
#define MPU9250_ACCEL_YOUT_L                     0x3e
#define MPU9250_ACCEL_ZOUT_H                     0x3f
#define MPU9250_ACCEL_ZOUT_L                     0x40

// Temp.
#define MPU9250_TEMP_OUT_H                       0x41
#define MPU9250_TEMP_OUT_L                       0x42

// Gyro.
#define MPU9250_GYRO_XOUT_H                      0x43
#define MPU9250_GYRO_XOUT_L                      0x44
#define MPU9250_GYRO_YOUT_H                      0x45
#define MPU9250_GYRO_YOUT_L                      0x46
#define MPU9250_GYRO_ZOUT_H                      0x47
#define MPU9250_GYRO_ZOUT_L                      0x48

// Ext. Sensor data
#define MPU9250_EXT_SENS_DATA_00                 0x49
#define MPU9250_EXT_SENS_DATA_01                 0x4a
#define MPU9250_EXT_SENS_DATA_02                 0x4b
#define MPU9250_EXT_SENS_DATA_03                 0x4c
#define MPU9250_EXT_SENS_DATA_04                 0x4d
#define MPU9250_EXT_SENS_DATA_05                 0x4e
#define MPU9250_EXT_SENS_DATA_06                 0x4f
#define MPU9250_EXT_SENS_DATA_07                 0x50
#define MPU9250_EXT_SENS_DATA_08                 0x51
#define MPU9250_EXT_SENS_DATA_09                 0x52
#define MPU9250_EXT_SENS_DATA_10                 0x53
#define MPU9250_EXT_SENS_DATA_11                 0x54
#define MPU9250_EXT_SENS_DATA_12                 0x55
#define MPU9250_EXT_SENS_DATA_13                 0x56
#define MPU9250_EXT_SENS_DATA_14                 0x57
#define MPU9250_EXT_SENS_DATA_15                 0x58
#define MPU9250_EXT_SENS_DATA_16                 0x59
#define MPU9250_EXT_SENS_DATA_17                 0x5a
#define MPU9250_EXT_SENS_DATA_18                 0x5b
#define MPU9250_EXT_SENS_DATA_19                 0x5c
#define MPU9250_EXT_SENS_DATA_20                 0x5d
#define MPU9250_EXT_SENS_DATA_21                 0x5e
#define MPU9250_EXT_SENS_DATA_22                 0x5f
#define MPU9250_EXT_SENS_DATA_23                 0x60

// I2C slave
#define MPU9250_I2C_SLV0_DO                      0x63
#define MPU9250_I2C_SLV1_DO                      0x64
#define MPU9250_I2C_SLV2_DO                      0x65
#define MPU9250_I2C_SLV3_DO                      0x66

#define MPU9250_I2C_MST_DELAY_CTRL               0x67


// Signal path
#define MPU9250_SIGNAL_PATH_RESET                0x68

// Motion detect
#define MPU9250_MOT_DETECT_CTRL                  0x69

// User
#define MPU9250_USER_CTRL                        0x6a // Bit 7 enable DMP, bit 3 reset DMP. See 0x6d

// Power management
#define MPU9250_PWR_MGMT_1                       0x6b
#define MPU9250_PWR_MGMT_2                       0x6c

// ...Looked for notes on DMP features, but Invensense docs were lacking.
// Found kriswiner's Arduino sketch for Basic AHRS, and found values/notes for
// Digital Motion Processing registers.
//
// See https://github.com/kriswiner/MPU-9250/blob/master/MPU9250BasicAHRS.ino
#define MPU9250_DMP_BANK                         0x6d
#define MPU9250_DMP_RW_PNT                       0x6e
#define MPU9250_DMP_REG                          0x6f
#define MPU9250_DMP_REG_1                        0x70
#define MPU9250_DMP_REG_2                        0x71

// FIFO Count
#define MPU9250_FIFO_COUNTH                      0x72
#define MPU9250_FIFO_COUNTL                      0x73
#define MPU9250_FIFO_R_W                         0x74

//WHO AM I
#define MPU9250_WHO_AM_I                    	 0x75

// Accel. offset
#define MPU9250_XA_OFFSET_H                      0x77
#define MPU9250_XA_OFFSET_L                      0x78
#define MPU9250_YA_OFFSET_H                      0x7a
#define MPU9250_YA_OFFSET_L                      0x7b
#define MPU9250_ZA_OFFSET_H                      0x7d
#define MPU9250_ZA_OFFSET_L                      0x7e

//Magnetometer Registers
#define AK8963_ADDRESS   0x0C
#define WHO_AM_I_AK8963  0x00 // should return 0x48
//#define INFO             0x01
#define AK8963_ST1       0x02  // data ready status bit 0
#define AK8963_XOUT_L	 0x03  // data
#define AK8963_XOUT_H	 0x04
#define AK8963_YOUT_L	 0x05
#define AK8963_YOUT_H	 0x06
#define AK8963_ZOUT_L	 0x07
#define AK8963_ZOUT_H	 0x08
#define AK8963_ST2       0x09  // Data overflow bit 3 and data read error status bit 2
#define AK8963_CNTL      0x0A  // Power down (0000), single-measurement (0001), self-test (1000) and Fuse ROM (1111) modes on bits 3:0
#define AK8963_ASTC      0x0C  // Self test control
#define AK8963_I2CDIS    0x0F  // I2C disable
#define AK8963_ASAX      0x10  // Fuse ROM x-axis sensitivity adjustment value
#define AK8963_ASAY      0x11  // Fuse ROM y-axis sensitivity adjustment value
#define AK8963_ASAZ      0x12  // Fuse ROM z-axis sensitivity adjustment value

#define MAG_BIAS_X  	0.022543f//-19.328068f
#define MAG_BIAS_Y		0.033066f//44.073853f
#define MAG_BIAS_Z      -0.468505f//-419.536224f
#define MAG_SCALE_X		1.231505f//1.047325f
#define MAG_SCALE_Y		0.897590f//0.980732f
#define MAG_SCALE_Z		0.931250f//0.919540f

#define M11 			 0.783025f
#define M12  			-0.001399f
#define M13  			-0.001613f
#define M21  			-0.0013993f
#define M22  			0.791298f
#define M23  			0.007157f
#define M31 			-0.001613f
#define M32 			0.007157f
#define M33  			0.785950f

bool mpu9250_init(const mpu9250_config_t *device_config);
void mpu9250_init_mag(void);
void mpu9250_init_mag();
bool mpu9250_test_connection(void);
bool mpu9250_mag_test_connection(void);
uint8_t mpu9250_accel_gyro_self_test(void);
uint8_t mpu9250_get_whoami(void);
uint8_t mpu9250_get_mag_whoami(void);
void mpu9250_accel_gyro_bias_cal();
void mpu9250_set_config();
void mpu9250_set_mag_config();
void mpu9250_set_mag_cal_data();

bool mpu9250_read_accel_data(int16_t *);
bool mpu9250_get_accel_data_gs(axis_float_t*);
bool mpu9250_read_gyro_data(int16_t *);
bool mpu9250_get_gyro_data_dps(axis_float_t*);
bool mpu9250_get_gyro_data_rps(axis_float_t *data);
bool mpu9250_read_mag_data(int16_t *);
bool mpu9250_get_mag_data_miligauss(axis_float_t*);

#endif /* _MPU9250_H_ */
