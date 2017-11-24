/*
 * nordic_hawk_pwm_motor.h
 *
 *  Created on: 16 oct. 2017
 *      Author: diego.aguirre
 */

#ifndef _NORDIC_HAWK_PWM_MOTOR_H_
#define _NORDIC_HAWK_PWM_MOTOR_H_

#include "nrf_drv_pwm.h"
#include "nrf_drv_clock.h"
#include "nordic_hawk_board.h"
#include "app_error.h"

#define TOP_PWM					32000//500Hz - 2ms period
#define PWM_INVERT_POLARITY 	0x8000
#define PWM_SCALER 				5
#define ONE_SHOT_125_MIN    	2000
#define ONE_SHOT_125_MAX    	4000


typedef enum{
	pwm_500hz = 0,
	pwm_1000hz,
}pwm_frequency_t;

//Motor throttle from 0 to 100
typedef struct{
	uint16_t motor1;
	uint16_t motor2;
	uint16_t motor3;
	uint16_t motor4;
}motor_values_t;

void pwm_init(pwm_frequency_t pwm_frequency);

/**
 * rate controller update to correct actual value and reduce error with set point(desired value).
 *
 * @param[in]  motor1					pwm value for motor1
 * @param[in]  motor2					pwm value for motor2
 * @param[in]  motor3					pwm value for motor3
 * @param[in]  motor4					pwm value for motor4
 * @ret							  	    true if all values are set properly, otherwise false
 */
bool update_motor_values(uint16_t motor1, uint16_t motor2, uint16_t motor3, uint16_t motor4);

#endif /* DRIVERS_NORDIC_HAWK_PWM_MOTOR_H_ */
