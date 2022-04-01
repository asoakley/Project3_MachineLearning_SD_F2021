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

extern volatile char Motors_Ready = 1;      // Flag for exclusive access to motors
extern volatile enum {IdleT, DelayT, Edge1, Corner1, Edge2, Corner2, Edge3, Corner3, DoneT} triangle_state = IdleT;
extern volatile enum {Idle8, Delay8, Straight1, LeftTurn, Straight2, RightTurn, Done8} figure8_state = Idle8;
extern volatile uint16_t TimerTicks;

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

void MotorsTimed(uint16_t duration, uint16_t leftspeed, char leftdirection, uint16_t rightspeed, char rightdirection){       // Eventually change this to an angle??
    Motors_Ready = 0;
    MotorsSimple(LEFT_OFF, FORWARD, RIGHT_OFF, FORWARD);
    MotorsSimple(leftspeed, leftdirection, rightspeed, rightdirection);
    SysTickCount_250ms(duration);
}

void TriangleFSM(void){
    if(Motors_Ready){
        switch (triangle_state){
            case IdleT:
                break;

            case DelayT:
                MotorsTimed(8,LEFT_OFF,FORWARD,RIGHT_OFF,FORWARD);
                triangle_state = Edge1;
                break;

            case Edge1:
                MotorsTimed(8,LEFT_SLOW,FORWARD,RIGHT_SLOW,FORWARD);
                triangle_state = Corner1;
                break;

            case Corner1:
                MotorsTimed(5,LEFT_SLOW,REVERSE,RIGHT_SLOW,FORWARD);
                triangle_state = Edge2;
                break;

            case Edge2:
                MotorsTimed(8,LEFT_SLOW,FORWARD,RIGHT_SLOW,FORWARD);
                triangle_state = Corner2;
                break;

            case Corner2:
                MotorsTimed(5,LEFT_SLOW,REVERSE,RIGHT_SLOW,FORWARD);
                triangle_state = Edge3;
                break;

            case Edge3:
                MotorsTimed(8,LEFT_SLOW,FORWARD,RIGHT_SLOW,FORWARD);
                triangle_state = Corner3;
                break;

            case Corner3:
                MotorsTimed(5,LEFT_SLOW,REVERSE,RIGHT_SLOW,FORWARD);
                triangle_state = DoneT;
                break;

            case DoneT:
                MotorsTimed(8,LEFT_OFF,FORWARD,RIGHT_OFF,FORWARD);
                triangle_state = IdleT;
                break;
            default: break;

        }
    }
}

void Figure8FSM(void){
    if(Motors_Ready){
       switch(figure8_state){
           case Idle8:
               break;

           case Delay8:
               MotorsTimed(8,LEFT_OFF,FORWARD,RIGHT_OFF,FORWARD);
               figure8_state = Straight1;
               break;

           case Straight1:
               MotorsTimed(8,LEFT_SLOW,FORWARD,RIGHT_SLOW,FORWARD);
               figure8_state = LeftTurn;
               break;

           case LeftTurn:
               MotorsTimed(9,LEFT_SLOW,FORWARD,RIGHT_MED,FORWARD);
               figure8_state = Straight2;
               break;

           case Straight2:
               MotorsTimed(8,LEFT_SLOW,FORWARD,RIGHT_SLOW,FORWARD);
               figure8_state = RightTurn;
               break;

           case RightTurn:
               MotorsTimed(9,LEFT_MED,FORWARD,RIGHT_SLOW,FORWARD);
               figure8_state = Done8;
               break;

           case DoneT:
               MotorsTimed(8,LEFT_OFF,FORWARD,RIGHT_OFF,FORWARD);
               figure8_state = Idle8;
               break;

           default: break;

       }
    }
}


