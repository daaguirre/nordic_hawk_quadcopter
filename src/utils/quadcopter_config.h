/*
 * quadcopter_config.h
 *
 *  Created on: 23 nov. 2017
 *      Author: diego.aguirre
 */

#ifndef UTILS_QUADCOPTER_CONFIG_H_
#define UTILS_QUADCOPTER_CONFIG_H_

#define CONTROLLER_PERIOD_MS    2 //quadcopter controller uptade period
#define MADGWICK_ITERATIONS		5 //number of iterations for madgwick algorithm

/*
 * Motor thrust varies from 0 to 2000 with current pwm configuration
 * Here a max thrust with a security margin is set
 */
#define MAX_MOTOR_THRUST 		1900
#define MIN_MOTOR_THRUST 		0

#endif /* UTILS_QUADCOPTER_CONFIG_H_ */
