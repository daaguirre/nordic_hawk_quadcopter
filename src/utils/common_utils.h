/*
 * math_utils.h
 *
 *  Created on: 13 nov. 2017
 *      Author: diego.aguirre
 */

#ifndef UTILS_COMMON_UTILS_H_
#define UTILS_COMMON_UTILS_H_

#include <stdint.h>

#define PI 3.14159265358979f

typedef union{
	uint16_t   	u16_value;
	uint8_t 	u8_array[2];
}byte_encoder_t;

//#undef max
#define max(a,b) ((a) > (b) ? (a) : (b))
//#undef min
#define min(a,b) ((a) < (b) ? (a) : (b))

float constrain(float value, const float minVal, const float maxVal);
void uint16_to_byte_array_encoder(uint16_t value, uint8_t * p_encoded_buffer);
uint16_t byte_array_to_uint16(uint8_t const *byte_array);

#endif /* UTILS_COMMON_UTILS_H_ */
