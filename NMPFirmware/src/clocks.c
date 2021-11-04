/*
 * clocks.c
 *
 *  Created on: Oct 6, 2021
 *      Author: austi
 */

#include <clocks.h>
#include <msp430.h>


void init_clocks(void){
    // MCLK = SMCLK = DCO = 1MHz

    // Select lowest DCOx and MODx
    DCOCTL = 0;
    // Set DCO freq to 1MHz and ACLK
    BCSCTL1 = CALBC1_1MHZ;
    DCOCTL = CALDCO_1MHZ;
}

