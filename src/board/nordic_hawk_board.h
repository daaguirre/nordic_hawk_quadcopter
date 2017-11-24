/*
 * nordic_hawk_board.h
 *
 *  Created on: 15 oct. 2017
 *      Author: diego.aguirre
 */

#include "boards.h"

#ifndef _NORDIC_HAWK_BOARD_H_
#define _NORDIC_HAWK_BOARD_H_

#define NORDIC_HAWK_PWM_MOTOR1 			20
#define NORDIC_HAWK_PWM_MOTOR2 			19
#define NORDIC_HAWK_PWM_MOTOR3 			18
#define NORDIC_HAWK_PWM_MOTOR4 			17

#define NORDIC_HAWK_I2C_SDA 			26
#define NORDIC_HAWK_I2C_SCL				27
#define MPU_I2C_INT						15


#define NORDIC_HAWK_SPI_MOSI			11
#define NORDIC_HAWK_SPI_CLK				12
#define EEPROM_SPI_CS					14
#define NORDIC_HAWK_SPI_MISO			16

void motor_config();

#endif /* BOARD_NORDIC_HAWK_BOARD_H_ */
