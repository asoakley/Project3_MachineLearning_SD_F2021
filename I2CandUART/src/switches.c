/*
 * switches.c
 *
 *  Created on: Oct 20, 2021
 *      Author: austi
 */

#include <gpio.h>
#include <motors.h>
#include <timers.h>
#include <IMU.h>

void PORT1_IRQHandler(void){
    if(P1IFG & SW1){        // Press SW1 to go forward, turn on LED
        P1IFG &= ~SW1;
        //MotorsSimple(LEFT_SLOW, FORWARD, RIGHT_SLOW, FORWARD);
        IMU_Read_Accel();
        P1OUT |= RED_LED;
    }

    if(P1IFG & SW2){
        P1IFG &= ~SW2;      // Press SW2 to go reverse, turn on LED
        //MotorsSimple(LEFT_SLOW, REVERSE, RIGHT_SLOW, REVERSE);
        P1OUT |= RED_LED;
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


