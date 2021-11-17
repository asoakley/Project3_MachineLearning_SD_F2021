/*
 * switches.c
 *
 *  Created on: Oct 20, 2021
 *      Author: austi
 */

#include <gpio.h>
#include <motors.h>
#include <timers.h>

//=================
// Globals
//=================

extern volatile enum {IdleT, DelayT, Edge1, Corner1, Edge2, Corner2, Edge3, Corner3, DoneT} triangle_state;
extern volatile enum {Idle8, Delay8, Straight1, LeftTurn, Straight2, RightTurn, Done8} figure8_state;

//==========================
// Button Switch Interrupts
//==========================

void PORT1_IRQHandler(void){
    if(P1IFG & SW1){
        P1IE &= ~SW1;
        P1IFG &= ~SW1;
        TIMER_A2->CCTL[1] &= ~CCIFG;
        triangle_state = DelayT;
        P1OUT |= RED_LED;
        TIMER_A2->CCR[1] = TA2R + TIMERA2_PERIOD;
        TIMER_A2->CCTL[1] |= CCIE;   // Capture mode, compare mode, enable CC interrupt
    }

    if(P1IFG & SW2){
        P1IE &= ~SW2;
        P1IFG &= ~SW2;
        TIMER_A2->CCTL[2] &= ~CCIFG;
        figure8_state = Delay8;
        P1OUT |= RED_LED;
        TIMER_A2->CCR[2] = TA2R + TIMERA2_PERIOD;
        TIMER_A2->CCTL[2] |= CCIE;   // Capture mode, compare mode, enable CC interrupt
    }
}

void PORT4_IRQHandler(void){    // Stop motors, turn off LED if any bump switch pressed
    if(P4IFG & BUMP0){      // Far right switch
        P4IFG &= ~BUMP0;
        MotorsSimple(LEFT_OFF, FORWARD, RIGHT_OFF, FORWARD);
        P1OUT &= ~RED_LED;
    }

    if(P4IFG & BUMP1){
        P4IFG &= ~BUMP1;
        MotorsSimple(LEFT_OFF, FORWARD, RIGHT_OFF, FORWARD);
        P1OUT &= ~RED_LED;
    }

    if(P4IFG & BUMP2){
        P4IFG &= ~BUMP2;
        MotorsSimple(LEFT_OFF, FORWARD, RIGHT_OFF, FORWARD);
        P1OUT &= ~RED_LED;
    }

    if(P4IFG & BUMP3){
        P4IFG &= ~BUMP3;
        MotorsSimple(LEFT_OFF, FORWARD, RIGHT_OFF, FORWARD);
        P1OUT &= ~RED_LED;
    }

    if(P4IFG & BUMP4){
        P4IFG &= ~BUMP4;
        MotorsSimple(LEFT_OFF, FORWARD, RIGHT_OFF, FORWARD);
        P1OUT &= ~RED_LED;
    }

    if(P4IFG & BUMP5){      // Far left switch
        P4IFG &= ~BUMP5;
        MotorsSimple(LEFT_OFF, FORWARD, RIGHT_OFF, FORWARD);
        P1OUT &= ~RED_LED;
    }

}


