/*
 * math_utils.c
 *
 *  Created on: 13 nov. 2017
 *      Author: diego.aguirre
 */

#include "common_utils.h"

#include <string.h>

float constrain(float value, const float minVal, const float maxVal)
{
  return min(maxVal, max(minVal,value));
}

void uint16_to_byte_array_encoder(uint16_t value, uint8_t * p_encoded_buffer)
{
	byte_encoder_t encoder;

	encoder.u16_value = value;
	memcpy(p_encoded_buffer, encoder.u8_array, 2);
}

uint16_t byte_array_to_uint16(uint8_t const * byte_array)
{
	byte_encoder_t encoder;
	memcpy(encoder.u8_array, byte_array, 2);

	return encoder.u16_value;
}
