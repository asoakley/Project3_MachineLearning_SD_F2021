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

extern volatile uint16_t TA2_CCR2_Count;
extern volatile enum {Idle, Delay, Edge1, Corner1, Edge2, Corner2, Edge3,Done} triangle_state;


//==========================
// Button Switch Interrupts
//==========================

void PORT1_IRQHandler(void){
    if(P1IFG & SW1){        // Press SW1 to go forward, turn on LED
        P4IE &= ~SW1;
        P1IFG &= ~SW1;
        TIMER_A2->CCTL[2] &= ~CCIFG;
        //MotorsSimple(LEFT_SLOW, FORWARD, RIGHT_SLOW, FORWARD);
        triangle_state = Delay;
        P1OUT |= RED_LED;
        TIMER_A2->CCR[2] = TA2R + TIMERA2_PERIOD;
        TIMER_A2->CCTL[2] |= CCIE;   // Capture mode, compare mode, enable CC interrupt
    }

    if(P1IFG & SW2){
        P4IE &= ~SW2;
        P1IFG &= ~SW2;      // Press SW2 to go reverse, turn on LED
        //MotorsSimple(LEFT_SLOW, REVERSE, RIGHT_SLOW, REVERSE);

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


