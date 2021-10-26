#include "msp.h"
#include <gpio.h>
#include <timers.h>
#include <motors.h>


/**
 * main.c
 */

// Globals



void main(void)
{
        Clock_Init48MHz();  // System clock at 48MHz, SMCLK at 12MHz
        Init_Ports();
        Init_Timers();
        MotorsSimple(LEFT_OFF, FORWARD, RIGHT_OFF, FORWARD);

        while(1){

            ;
        }
}
