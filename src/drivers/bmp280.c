/*
 * bmp280.c
 *
 *  Created on: 26 ago. 2017
 *      Author: Diego Aguirre
 */

#include <stdint.h>
#include <stdbool.h>

#include "bmp280_reg.h"
#include "bmp280.h"
#include "nrf_delay.h"
#include "i2cdev.h"

static struct bmp280_t *p_bmp280; /**< pointer to BMP280 */

/*!
 *	@brief This function is used for initialize
 *	the bus read and bus write functions
 *  and assign the chip id and I2C address of the BMP280 sensor
 *	chip id is read in the register 0xD0 bit from 0 to 7
 *
 *	@param *bmp280 structure pointer.
 *
 *	@note While changing the parameter of the p_bmp280
 *	@note consider the following point:
 *	Changing the reference value of the parameter
 *	will changes the local copy or local reference
 *	make sure your changes will not
 *	affect the reference value of the parameter
 *	(Better case don't change the reference value of the parameter)
 *
 *
 *
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval -1 -> Error
 *
 *
 */
bool bmp280_init(struct bmp280_t *bmp280)
{
	/* variable used to return communication result*/
	int8_t com_rslt;
	uint8_t data = 0;
	uint8_t chip_id_read_count = 5;

	p_bmp280 = bmp280;/* assign BMP280 ptr */
	p_bmp280->dev_addr = BMP280_I2C_ADDRESS1;
	p_bmp280->read_reg = i2cdev_readBytes;
	p_bmp280->write_reg = i2cdev_writeBytes;
	p_bmp280->delay_msec = nrf_delay_ms;

	while (chip_id_read_count > 0) {
		/* read chip id */
		com_rslt = p_bmp280->read_reg(p_bmp280->dev_addr,
				BMP280_CHIP_ID_REG, 1, &data);
		/* Check for the correct chip id */
		if ((data == BMP280_CHIP_ID1)
			|| (data == BMP280_CHIP_ID2)
			|| (data == BMP280_CHIP_ID3))
			break;
		chip_id_read_count--;
		/* Delay added concerning the low speed of power up system to
		facilitate the proper reading of the chip ID */
		p_bmp280->delay_msec(1);
	}

	/*assign chip ID to the global structure*/
	p_bmp280->chip_id = data;

//	if (com_rslt == 0) {
//		/* readout bmp280 calibration parameter structure */
//		com_rslt += bmp280_get_calib_param();
//	}
	return com_rslt != 0;
}
