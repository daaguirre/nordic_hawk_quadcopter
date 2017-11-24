/*
 * real_time_debugger.h
 *
 *  Created on: 29 ago. 2017
 *      Author: Diego Aguirre
 */

#ifndef _REAL_TIME_DEBUGGER_H_
#define _REAL_TIME_DEBUGGER_H_

#include <stdint.h>

void rtt_print_float(char *text, float value);
void rtt_print_int(const char *text, uint16_t value);
void rtt_print_uint32(const char *text, uint32_t value);
void rtt_println(char *text);

#endif /* _REAL_TIME_DEBUGGER_H_ */
