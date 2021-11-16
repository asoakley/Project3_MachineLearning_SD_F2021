/*
 * motors.c
 *
 *  Created on: Oct 20, 2021
 *      Author: austi
 */

#include "msp.h"
#include <timers.h>
#include <motors.h>
#include <gpio.h>

extern volatile uint16_t TA2_CCR1_Count;
extern volatile char Motors_Ready = 1;      // Flag for exclusive access to motors
extern volatile enum {Idle, Delay, Edge1, Corner1, Edge2, Corner2, Edge3,Done} triangle_state = Idle;


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

void SpinInPlace(uint16_t duration, uint16_t speed, uint16_t direction){       // Eventually change this to an angle??
    Motors_Ready = 0;
    MotorsSimple(LEFT_OFF, FORWARD, RIGHT_OFF, FORWARD);
    MotorsSimple(speed, direction, speed, ~direction);
    TIMER_A2->CCR[1] = TA2R + duration;   // Timer A2 Period is 0.25 sec
    //TIMER_A2->CCTL[1] |= CCIE;   // Capture mode, compare mode, enable CC interrupt, clear CCIFG
}

void StraightTimed(uint16_t duration, uint16_t speed, uint16_t direction){
    Motors_Ready = 0;
    MotorsSimple(LEFT_OFF, FORWARD, RIGHT_OFF, FORWARD);
    MotorsSimple(speed, direction, speed, direction);
    TIMER_A2->CCR[1] = TA2R + duration;   // Timer A2 Period is 0.25 sec
    //TIMER_A2->CCTL[1] |= CCIE;   // Capture mode, compare mode, enable CC interrupt, clear CCIFG
}

void TriangleFSM(void){
    if(Motors_Ready){
        switch (triangle_state){
            case Idle:
                break;

            case Delay:
                StraightTimed(TIMERA2_PERIOD,LEFT_OFF,FORWARD);
                triangle_state = Edge1;
                break;

            case Edge1:
                StraightTimed(TIMERA2_PERIOD,LEFT_SLOW,FORWARD);
                triangle_state = Corner1;
                break;

            case Corner1:
                SpinInPlace(TIMERA2_QUARTER,LEFT_SLOW,COUNTERCLOCKWISE);
                triangle_state = Edge2;
                break;

            case Edge2:
                StraightTimed(TIMERA2_PERIOD,LEFT_SLOW,FORWARD);
                triangle_state = Corner2;
                break;

            case Corner2:
                SpinInPlace(TIMERA2_QUARTER,LEFT_SLOW,COUNTERCLOCKWISE);
                triangle_state = Edge3;
                break;

            case Edge3:
                StraightTimed(TIMERA2_PERIOD,LEFT_SLOW,FORWARD);
                triangle_state = Done;
                break;

            case Done:
                StraightTimed(TIMERA2_PERIOD,LEFT_OFF,FORWARD);
                triangle_state = Idle;
                break;
            default: break;


        }
    }
}

