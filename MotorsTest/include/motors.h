/*
 * motors.h
 *
 *  Created on: Oct 20, 2021
 *      Author: austi
 */

#include "msp.h"

#ifndef INCLUDE_MOTORS_H_
#define INCLUDE_MOTORS_H_

#define MOTOR_PWM_PERIOD            7500

#define LEFT_SLOW                   1500
#define RIGHT_SLOW                  1500

#define LEFT_OFF                    0
#define RIGHT_OFF                   0

#define FORWARD                     1
#define REVERSE                     0


// Functions

void MotorsSimple(uint16_t leftSpeed, char leftDirection, uint16_t rightSpeed, char rightDirection);


#endif /* INCLUDE_MOTORS_H_ */
