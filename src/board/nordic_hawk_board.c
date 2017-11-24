/*
 * nordic_hawk_board.c
 *
 *  Created on: 16 oct. 2017
 *      Author: diego.aguirre
 */

#include "nordic_hawk_board.h"

void motor_config(void)
{
	nrf_gpio_cfg_output(NORDIC_HAWK_PWM_MOTOR1);
	nrf_gpio_cfg_output(NORDIC_HAWK_PWM_MOTOR2);
	nrf_gpio_cfg_output(NORDIC_HAWK_PWM_MOTOR3);
	nrf_gpio_cfg_output(NORDIC_HAWK_PWM_MOTOR4);
}
