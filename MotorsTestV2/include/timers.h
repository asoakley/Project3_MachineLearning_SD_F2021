/*
 * timers.h
 *
 *  Created on: Oct 16, 2021
 *      Author: austi
 */

#include "msp.h"

#ifndef INCLUDE_TIMERS_H_
#define INCLUDE_TIMERS_H_

// Macros

#define TIMERA2_PERIOD      65535
#define TIMERA2_HALF        32767
#define TIMERA2_QUARTER     16383

// functions

void Clock_Init48MHz(void);
void Init_Timers(void);
void Init_TimerA0(uint16_t period, uint16_t duty_left, uint16_t duty_right);
void Init_TimerA2(uint16_t period);

#endif /* INCLUDE_TIMERS_H_ */
