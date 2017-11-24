/*
 * real_time_debugger.c
 *
 *  Created on: 29 ago. 2017
 *      Author: Diego Aguirre
 */

#include <stdio.h>
#include <SEGGER_RTT.h>

/*
 * print float number through RTT console
 * Max 30 characters
 */
void rtt_print_float(const char *text, float value)
{
	char buffer[30];

	sprintf(buffer, "%s %f \n", text, value);
	SEGGER_RTT_WriteString(0, buffer);
}

/*
 * print int number through RTT console
 * Max 30 characters
 */
void rtt_print_int(const char *text, uint16_t value)
{
	char buffer[30];

	sprintf(buffer, "%s %d \n", text, value);
	SEGGER_RTT_WriteString(0, buffer);
}

/*
 * print int number through RTT console
 * Max 30 characters
 */
void rtt_print_uint32(const char *text, uint32_t value)
{
	char buffer[30];

	sprintf(buffer, "%s %lu \n", text, (unsigned long)value);
	SEGGER_RTT_WriteString(0, buffer);
}

/*
 * print line at RTT console
 */
void rtt_println(char *text)
{
	SEGGER_RTT_WriteString(0, text);
}

