/*
 * switches.c
 *
 *  Created on: Oct 20, 2021
 *      Author: austi
 */

#include <gpio.h>

void PORT1_IRQHandler(void){
    if(P1IFG & SW1){
        P1IFG &= ~SW1;
        P1OUT |= RED_LED;
    }

    if(P1IFG & SW2){
        P1IFG &= ~SW2;
        P1OUT &= ~RED_LED;
    }
}

