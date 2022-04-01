/*
 * MotorsTest
 * motors.c
 *
 *  Created on: Oct 20, 2021
 *      Author: austi
 */

#include "msp.h"
#include <timers.h>
#include <motors.h>
#include <gpio.h>

void MotorsSimple(uint16_t leftSpeed, char leftDirection, uint16_t rightSpeed, char rightDirection){
    if((leftSpeed <= MOTOR_PWM_PERIOD) && (rightSpeed <= MOTOR_PWM_PERIOD)){
        TIMER_A0->CCR[4] = leftSpeed;
        TIMER_A0->CCR[3] = rightSpeed;
    }
    else{
        TIMER_A0->CCR[4] = LEFT_OFF;
        TIMER_A0->CCR[3] = RIGHT_OFF;
    }

    if(leftDirection) P5OUT &= ~LDIR;       // Left forward
    else P5OUT |= LDIR;                     // Left reverse

    if(rightDirection) P5OUT &= ~RDIR;      // Right forward
    else P5OUT |= RDIR;                     // Right reverse

}
